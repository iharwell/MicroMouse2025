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

        struct MeasuredMotion final
        {
            float linearSpeedMps{};
            float angularSpeedRadps{};
        };

        struct ModeState final
        {
            std::uint32_t sequence{};
            std::uint32_t tickStartUs{};
            std::uint32_t commandApplyTimeUs{};
            std::uint32_t dtUs{};
            float dtSeconds{};
            PoseEstimate estimate{};
            MeasuredMotion measured{};
            DriveTelemetry driveTelemetry{};
            SensorSnapshot sensors{};
            bool estimatorHealthy{ true };
            bool overrun{};
            const char* faultReason{};
        };

        struct PauseContext final
        {
            ModeState stateEstimate{};
            const char* reason{};
        };

        struct PauseDisposition final
        {
            enum class Action : std::uint8_t
            {
                Resume,
                Complete,
                StopByRuntime
            } action{ Action::Resume };

            bool resetClockOnResume{ true };
            const char* stopReason{};

            static PauseDisposition Resume() noexcept;
            static PauseDisposition Complete() noexcept;
            static PauseDisposition StopByRuntime(const char* reason) noexcept;
        };

        class TickServices;

        using ModeWorkCallback = ControlVector (*)(
            void* context,
            std::uint32_t loopEndTimeUs,
            const ModeState& state,
            TickServices& services);

        using PauseCallback = PauseDisposition (*)(
            void* context,
            const PauseContext& pause);

        struct PauseRequest final
        {
            PauseCallback onPauseGranted{};
            const char* reason{};
            float maxAbsLinearSpeed{ -1.0f };
            float maxAbsAngularSpeed{ -1.0f };
            std::uint8_t consecutiveSettledTicks{};
            bool flushLogsBeforeGrant{ true };
            bool resetClockOnResume{ true };
        };

        struct ModeCallbacks final
        {
            ModeWorkCallback onModeWork{};
            void* context{};
        };

        class TickServices final
        {
        public:
            void Fault(const char* reason) noexcept;
            // Pause is the only sanctioned way to leave strict periodic cadence for non-periodic
            // work. Do not replace it with caller-driven "tick once, then return to me" control.
            void RequestPause(const PauseRequest& request) noexcept;
            void RequestEndLoop() noexcept;
            void SetNextModeWorkCallbacks(const ModeCallbacks& callbacks) noexcept;
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

    private:
        friend class Drive;
        friend class SharedRobotRuntime;
        friend class StartupCalibration;
        friend class WallTouch;

        struct ObservedTickState final
        {
            std::uint32_t sequence{};
            std::uint32_t tickStartUs{};
            std::uint32_t dtUs{};
            float dtSeconds{};
            PoseEstimate estimate{};
            MeasuredMotion measured{};
            DriveTelemetry driveTelemetry{};
            SensorSnapshot sensors{};
            bool estimatorHealthy{ true };
            bool overrun{};
            const char* faultReason{};
        };

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
        static PoseEstimate ProjectEstimate(
            const PoseEstimate& estimate,
            std::uint32_t projectionAnchorUs,
            std::uint32_t commandApplyTimeUs) noexcept;

        void RunSessionStartWallSensorAdcProbe() noexcept;
        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;
        bool ValidateSessionOptions(const SessionOptions& options) const noexcept;
        bool SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept;
        void ResetLatchedRequests() noexcept;
        bool ApplyControlAtTickStart(const ControlVector& control) noexcept;
        bool ExecuteSensingUpdate(ObservedTickState& observed);
        bool CaptureTickState(ObservedTickState& observed);
        const ModeState* CurrentModeState() const noexcept;
        ModeState BuildModeState(
            const ObservedTickState& observed,
            std::uint32_t projectionAnchorUs,
            std::uint32_t commandApplyTimeUs,
            bool overrunBeforeModeWork) const noexcept;
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
        bool WaitForPauseSettlement(const PauseRequest& request, ModeState& settledState);
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
        ObservedTickState _observedScratch{};
        ModeState _callbackModeState{};
        bool _callbackModeStateValid{};
        PauseContext _pauseContextScratch{};
    };
}
