#pragma once

#include "MazeMapRuntimeCore.h"

#include <cstdint>

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class LoopController final
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

        // The loop owns the startup stationary warmup and does not invoke the mode callback
        // until this tick sequence number is reached.
        static constexpr std::uint32_t kInitialModeCallbackTick = 250U;

        struct ControlVector final
        {
            enum class Kind : std::uint8_t
            {
                Brake,
                Velocity,
                OpenLoopRaw,
                NoChange
            } kind{ Kind::Brake };

            float linearTarget{};
            float angularTarget{};
            float leftOpenLoop{};
            float rightOpenLoop{};

            static ControlVector BrakeCommand() noexcept;
            static ControlVector HoldZeroVelocityCommand() noexcept;
            static ControlVector VelocityCommand(float linearTarget, float angularTarget) noexcept;
            static ControlVector OpenLoopCommand(float leftCommand, float rightCommand) noexcept;
            static ControlVector NoChangeCommand() noexcept;
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
            DiagnosticSensorSnapshot diagnosticSensors{};
            bool hasDiagnosticSensors{};
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
            void RequestPause(const PauseRequest& request) noexcept;
            void RequestEndLoop() noexcept;
            void SetNextModeWorkCallback(ModeWorkCallback callback) noexcept;

        private:
            friend class LoopController;
            explicit TickServices(LoopController& owner) noexcept;

            LoopController* _owner{};
        };

        LoopController() = default;

        bool BeginSession(const SessionOptions& options, const ModeCallbacks& callbacks);
        SessionResult Run();
        void EndSession();

        bool SessionActive() const noexcept;
        const TimingDiagnostics& LastDiagnostics() const noexcept;
        const ControlVector& LastAppliedCommand() const noexcept;

    private:
        friend class SharedRobotRuntime;

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
            DiagnosticSensorSnapshot diagnosticSensors{};
            bool hasDiagnosticSensors{};
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
            ModeWorkCallback nextModeWorkCallback{};
        };

        static constexpr std::uint8_t kTimingFlagResumedFromPause = 1U << 0;
        static constexpr std::uint8_t kTimingFlagPausePending = 1U << 1;
        static constexpr std::uint8_t kTimingFlagRuntimeStopPending = 1U << 2;

        static std::uint16_t RelativeTickUs(std::uint32_t tickStartUs, std::uint32_t timestampUs) noexcept;
        static bool IsZeroVelocityCommand(const ControlVector& command) noexcept;
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
        ControlVector NormalizeQueuedControl(const ControlVector& candidate) const noexcept;
        void ApplyControlAtTickStart(const ControlVector& control, float dtSeconds);
        bool ExecuteSensingUpdate(ObservedTickState& observed, TimingDiagnostics& timing);
        bool CaptureMissionTickState(ObservedTickState& observed, TimingDiagnostics& timing);
        bool CaptureDiagnosticTickState(ObservedTickState& observed, TimingDiagnostics& timing);
        bool CaptureSelectedTickState(ObservedTickState& observed, TimingDiagnostics& timing);
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
        void RecordModeReturnTiming(std::uint32_t tickStartUs) noexcept;
        void RecordPostServiceTiming(std::uint32_t tickStartUs) noexcept;
        void FinalizeTiming(std::uint32_t tickStartUs) noexcept;
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
        bool _sessionBegun{};
        bool _sessionActive{};
        bool _resumePending{};
        bool _publishedTimingValid{};
        std::uint32_t _tickCount{};
        std::uint32_t _lastTickStartUs{};
        std::uint32_t _nextSyncTargetUs{};
        ControlVector _queuedControl{};
        ControlVector _appliedControl{};
        bool _sessionStartWallSensorAdcProbePending{};
        TimingDiagnostics _timingBuffers[2]{};
        std::uint8_t _publishedTimingIndex{ 0U };
        std::uint8_t _workingTimingIndex{ 1U };
        LatchedRequests _requests{};
        DeferredTerminalOutcome _deferredTerminalOutcome{ DeferredTerminalOutcome::None };
        const char* _deferredTerminalReason{};
        ObservedTickState _observedScratch{};
        PauseContext _pauseContextScratch{};
    };
}
