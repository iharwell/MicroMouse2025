#pragma once

#include "CommandVector.h"
#include "Defines.h"
#include "DriveBaseTrackingTuning.h"
#include "DriveTelemetry.h"
#include "PlantModel.h"
#include "VehicleState.h"

#include <cstdint>

namespace MazeMap
{
    class EXPORT DriveBase final
    {
    public:
        explicit DriveBase(
            const MazeMap::PlantModel& plant,
            const MazeMap::VehicleState& runtimeState,
            const MazeMap::DriveBaseTrackingTuning& feedbackTuning) noexcept;

        DriveBase(const DriveBase&) = delete;
        DriveBase& operator=(const DriveBase&) = delete;
        DriveBase(DriveBase&&) = delete;
        DriveBase& operator=(DriveBase&&) = delete;

        void ClearCommandEvidence() noexcept;

        MazeMap::App::Internal::CommandVector ProposeBodyTick(
            float targetForwardMps,
            float targetYawRateRadps,
            float targetForwardAccelMps2,
            float targetYawAccelRadps2,
            float targetYawRad) noexcept;

        const DriveTelemetry& LastTelemetry() const noexcept;

    private:
        DriveTelemetry BuildBaseTelemetry(
            std::uint16_t commandKindFlags,
            float targetForwardMps,
            float targetYawRateRadps,
            float targetForwardAccelMps2,
            float targetYawAccelRadps2,
            float targetYawRad) noexcept;

        const MazeMap::PlantModel& _plant;
        const MazeMap::VehicleState& _runtimeState;
        const MazeMap::DriveBaseTrackingTuning& _feedbackTuning;
        DriveTelemetry _lastTelemetry{};
        std::uint32_t _nextProposalSequenceId = 1U;
    };
}
