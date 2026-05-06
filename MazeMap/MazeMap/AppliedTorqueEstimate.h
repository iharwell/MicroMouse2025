#pragma once

namespace MazeMap
{
    // Plant-side estimate of the wheel-bank torque actually reaching the contact model for one control cycle.
    struct AppliedTorqueEstimate
    {
        float leftAppliedBankTorqueNm = 0.0f;
        float rightAppliedBankTorqueNm = 0.0f;
        bool leftCurrentLimited = false;
        bool rightCurrentLimited = false;
        bool batteryVoltageAvailable = false;
    };
}
