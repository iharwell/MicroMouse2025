#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapRuntimeCore.h"

#include <cstdint>
#include <type_traits>

class DriveBase;
class SensorSuite;
class DiagnosticSensorSuite;

namespace MazeMap
{
    class Maze;

    namespace mmlog
    {
        class MmLogLogger;
    }
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class LoopController final
    {
    public:
        struct VehicleState;

        struct TickTiming final
        {
            std::uint32_t tickStartUs{};
            std::uint32_t dtUs{};

            std::uint16_t tActuationAppliedUs{};
            std::uint16_t tEncoderDoneUs{};
            std::uint16_t tFrontReadyUs{};
            std::uint16_t tLeftReadyUs{};
            std::uint16_t tRightReadyUs{};
            std::uint16_t tImuDoneUs{};
            std::uint16_t tEstimatorDoneUs{};
            std::uint16_t tModeReturnUs{};
            std::uint16_t tPostServiceDoneUs{};
            std::uint16_t overrunUs{};

            std::uint8_t flags{};
        };

        struct CaptureOptions final
        {
            enum class WallMask : std::uint8_t
            {
                None = 0x00,
                Front = 0x01,
                Left = 0x02,
                Right = 0x04,
                All = 0x07
            } walls{ WallMask::All };

            bool readGyro{ true };
            bool readAccel{ true };
        };

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

        struct PauseRequest final
        {
            const char* reason{};

            float maxAbsLinearSpeed{ -1.0f };
            float maxAbsAngularSpeed{ -1.0f };
            std::uint8_t consecutiveSettledTicks{};

            bool flushServicesBeforeGrant{ true };
            bool resetClockOnResume{ true };
        };

        struct HeavyWorkResult final
        {
            enum class Action : std::uint8_t
            {
                Resume,
                Complete,
                Fault
            } action{ Action::Resume };

            bool resetClockOnResume{ true };
            const char* faultReason{};

            static HeavyWorkResult Resume() noexcept;
            static HeavyWorkResult Complete() noexcept;
            static HeavyWorkResult Fault(const char* reason) noexcept;
        };

        struct SessionResult final
        {
            enum class Status : std::uint8_t
            {
                Running,
                Completed,
                Faulted
            } status{ Status::Completed };

            std::uint32_t tickCount{};
            const char* faultReason{};
            bool pauseGranted{};
            bool resumedFromPause{};
        };

        struct RuntimeBundle final
        {
            SharedRobotRuntime& shared;
            DriveBase& driveBase;
            SensorSuite* missionSensors{};
            DiagnosticSensorSuite* diagnosticSensors{};
            MazeMap::Maze* maze{};

            MazeMap::mmlog::MmLogLogger* primaryLog{};
            MazeMap::mmlog::MmLogLogger* timingLog{};
            MazeMap::mmlog::MmLogLogger* utilityLog{};
        };

        struct SessionConfig final
        {
            BootModeId bootModeId{};
            const BootModeDescriptor* descriptor{};
            const char* sessionName{};

            std::uint32_t controlPeriodUs{};
            std::uint32_t idleSleepUs{ 20U };

            enum class StartupCommandPolicy : std::uint8_t
            {
                Brake,
                HoldZeroVelocity,
                UseProvidedInitialCommand
            } startupCommandPolicy{ StartupCommandPolicy::Brake };

            ControlVector initialCommand{};

            CaptureOptions defaultCapture{};
            bool allowDynamicCaptureOverride{ true };

            enum class GuardPolicy : std::uint8_t
            {
                None,
                Workspace,
                Boundary,
                Custom
            } guardPolicy{ GuardPolicy::None };

            enum class GuardSeverity : std::uint8_t
            {
                ReportOnly,
                BrakeAndContinue,
                TerminalFault
            } guardSeverity{ GuardSeverity::BrakeAndContinue };

            enum class InstrumentationLevel : std::uint8_t
            {
                Minimal,
                Standard,
                Detailed
            } instrumentation{ InstrumentationLevel::Standard };

            enum class ActuationPolicy : std::uint8_t
            {
                VelocityBrake,
                VelocityBrakeOpenLoop
            } actuationPolicy{ ActuationPolicy::VelocityBrake };

            struct PauseDefaults final
            {
                enum class SettleActuation : std::uint8_t
                {
                    Brake,
                    HoldZeroVelocity
                } settleActuation{ SettleActuation::Brake };

                float maxAbsLinearSpeed{ 0.01f };
                float maxAbsAngularSpeed{ 0.05f };
                std::uint8_t consecutiveSettledTicks{ 2U };

                bool flushServicesBeforeGrant{ true };
                bool resetClockOnResume{ true };
            } pauseDefaults{};

            bool serviceWaitState{ true };
            bool serviceSlackState{ true };
            bool maintainTickSequence{ true };
            bool snapshotDriveTelemetry{ false };
            bool deriveMeasuredKinematics{ false };
        };

        struct MeasuredKinematics final
        {
            float leftVelocityMps{};
            float rightVelocityMps{};
            float linearSpeedMps{};
            float angularSpeedRadps{};
        };

        struct VehicleState final
        {
            std::uint32_t sequence{};
            std::uint32_t tickStartUs{};
            std::uint32_t dtUs{};
            float dtSeconds{};

            TickTiming timing{};
            ControlCycleTiming controlCycleTiming{};
            CaptureOptions captureUsed{};

            ControlVector appliedControl{};

            PoseEstimate estimate{};
            MeasuredKinematics measured{};
            DriveTelemetry driveTelemetry{};
            SensorSnapshot sensors{};
            DiagnosticSensorSnapshot diagnosticSensors{};
            bool hasDiagnosticSensors{};

            bool estimatorHealthy{ true };
            bool guardHealthy{ true };
            bool resumedFromPause{ false };
            bool overrun{ false };
            const char* faultReason{};
        };

        struct PauseContext final
        {
            VehicleState stateEstimate{};
            const char* reason{};
        };

        class TickServices;

        class IMode
        {
        public:
            virtual ~IMode() = default;

            virtual bool OnSessionBegin(const VehicleState& initial) = 0;

            virtual ControlVector Step(
                std::uint32_t availableComputeUs,
                const VehicleState& state,
                TickServices& services) = 0;

            virtual HeavyWorkResult OnPauseGranted(const PauseContext& pause)
            {
                (void)pause;
                return HeavyWorkResult::Resume();
            }

            virtual void OnSessionEnd(const SessionResult& result) = 0;
            virtual void ServiceWaitState() {}
            virtual void ServiceSlackState() {}
        };

        class TickServices final
        {
        public:
            void Fault(const char* reason);
            void RequestPauseForHeavyWork() noexcept;
            void RequestPauseForHeavyWork(const PauseRequest& request) noexcept;
            void RequestEndLoop() noexcept;
            void SetNextTickCaptureOptions(const CaptureOptions& options) noexcept;

        private:
            friend class LoopController;
            explicit TickServices(LoopController& owner) noexcept;

            LoopController* _owner{};
        };

        bool BeginSession(
            const SessionConfig& config,
            RuntimeBundle& runtime,
            IMode& mode);

        SessionResult Run();
        SessionResult RunOneTick();
        template <typename Callback>
        SessionResult RunOneTickWithCallback(Callback&& callback)
        {
            using CallbackType = std::remove_reference_t<Callback>;

            if (_tickStepCallback != nullptr)
            {
                SessionResult result{};
                result.status = SessionResult::Status::Faulted;
                result.tickCount = _tickCount;
                result.faultReason = "LoopController temporary tick callback already installed";
                return _sessionActive ? FinishSession(result) : result;
            }

            _tickStepContext = const_cast<void*>(static_cast<const void*>(&callback));
            _tickStepCallback = [](void* context,
                                   std::uint32_t availableComputeUs,
                                   const VehicleState& state,
                                   TickServices& services)
                -> ControlVector
            {
                return (*static_cast<CallbackType*>(context))(availableComputeUs, state, services);
            };

            const SessionResult result = RunOneTick();
            _tickStepContext = nullptr;
            _tickStepCallback = nullptr;
            return result;
        }
        void EndSession();

        bool SessionActive() const noexcept;

    private:
        static constexpr std::uint8_t kTimingFlagResumedFromPause = 1U << 0;
        static constexpr std::uint8_t kTimingFlagPausePending = 1U << 1;
        static constexpr std::uint8_t kTimingFlagCaptureOverride = 1U << 2;

        struct LatchedRequests final
        {
            const char* faultReason{};
            bool endRequested{};
            bool pauseRequested{};
            PauseRequest pauseRequest{};
            bool captureOverrideRequested{};
            CaptureOptions nextCapture{};
        };

        using TickStepCallback = ControlVector (*)(
            void* context,
            std::uint32_t availableComputeUs,
            const VehicleState& state,
            TickServices& services);

        static std::uint16_t RelativeTickUs(std::uint32_t tickStartUs, std::uint32_t timestampUs) noexcept;
        static bool IsZeroVelocityCommand(const ControlVector& command) noexcept;
        static bool IsFullCapture(const CaptureOptions& options) noexcept;

        bool ValidateSessionConfig(const SessionConfig& config) const noexcept;
        VehicleState BuildInitialState() const noexcept;
        ControlVector ResolveStartupCommand() const noexcept;
        ControlVector NormalizeQueuedControl(const ControlVector& candidate) const noexcept;
        ControlVector InvokeTickStep(
            std::uint32_t availableComputeUs,
            const VehicleState& state,
            TickServices& services);
        void ApplyControlAtTickStart(const ControlVector& control, float dtSeconds);
        void ApplyTerminalActuation() noexcept;
        void ApplyFaultActuation() noexcept;
        bool WaitForTickBoundaryAndService();
        void ServiceBackgroundWork(bool waitState) noexcept;
        void ServiceSlackState() noexcept;
        bool CaptureTickState(VehicleState& state);
        bool CaptureMissionTickState(VehicleState& state);
        bool CaptureDiagnosticTickState(VehicleState& state);
        bool CaptureSelectedTickState(VehicleState& state);
        bool CaptureTickStateWithResolvedSensors(VehicleState& state, bool stationaryHint);
        bool SupportsCaptureOptions(const CaptureOptions& options) const noexcept;
        bool ResolvePauseRequest(SessionResult& result);
        bool WaitForPauseSettlement(const PauseRequest& request, VehicleState& settledState);
        SessionResult FinishSession(SessionResult result);
        std::uint32_t ComputeRemainingBudgetUs(std::uint32_t tickStartUs) const noexcept;
        bool ShouldTreatAppliedControlAsStationary() const noexcept;
        void ResetLatchedRequests() noexcept;
        void RecordModeReturnTiming(VehicleState& state) const noexcept;
        void RecordPostServiceTiming(VehicleState& state) const noexcept;
        void RecordOverrun(VehicleState& state) const noexcept;

        SessionConfig _config{};
        RuntimeBundle* _runtime{};
        IMode* _mode{};
        bool _sessionBegun{};
        bool _sessionActive{};
        bool _sessionEndNotified{};
        bool _captureOverrideActive{};
        bool _resumePending{};
        std::uint32_t _tickCount{};
        unsigned long _lastTickStartUs{};
        ControlVector _queuedControl{};
        ControlVector _appliedControl{};
        CaptureOptions _captureForNextTick{};
        LatchedRequests _requests{};
        const char* _faultReason{};
        void* _tickStepContext{};
        TickStepCallback _tickStepCallback{};
    };
}
