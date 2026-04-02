#pragma once
#include "Defines.h"

namespace MazeMap::FrontWallCharacterizationConfig
{
    // [Low] Short pins 39 and 40 at startup to enter the persistent front-wall characterization routine. These pins
    // are selector-only and are not sampled as part of the stored curve.
    constexpr uint8_t kModeSelectPinA = 39U;
    constexpr uint8_t kModeSelectPinB = 40U;
    // [Medium] Control period for the reverse characterization run. Keep this fast enough for smooth wheel control and
    // fine encoder-based spacing along the captured curve.
    constexpr unsigned long kControlPeriodUs = 1000UL;
    // [Medium] Initial settle before the reverse sweep starts. Increase if placement against the wall still rings in
    // the chassis or if the operator needs more time after letting go.
    constexpr uint16_t kStartupSettleMs = 400U;
    // [Medium] Once the SD card is present, wait this long before starting the characterization flow so the operator
    // can finish installing the card and clear their hands from the robot.
    constexpr uint16_t kPostSdReadyDelayMs = 15000U;
    // [Medium] Reverse speed used while backing away from the wall. Keep this slow so the stored curve is dominated by
    // the front-sensor response rather than drivetrain transients.
    constexpr float kReverseSpeedMps = 0.08f;
    // [Medium] Acceleration limit while ramping into the reverse characterization speed.
    constexpr float kReverseAccelMps2 = 0.20f;
    // [Medium] Angular-rate limit used by the heading hold during the reverse sweep.
    constexpr float kMaxAngularCommandRadps = 2.0f;
    // [Medium] Distance spacing between stored curve samples. Lower for denser templates; raise if EEPROM space is
    // needed elsewhere.
    constexpr float kStoredDistanceStepM = 0.001f;
    // [Medium] Maximum reverse travel allowed while searching for the collapse-to-zero region.
    constexpr float kMaxReverseTravelM = 0.14f;
    // [Medium] Differential-light threshold treated as collapsed-to-zero during this dark-room characterization mode.
    constexpr float kCollapsedDifferentialLightThreshold = 0.0005f;
    // [Medium] Require the collapsed condition for several consecutive control samples before ending the sweep so a
    // single noisy read does not truncate the stored curve.
    constexpr uint8_t kCollapsedConsecutiveSamples = 12U;
    // [Low] Ignore collapse-to-zero until the robot has backed off by at least this much, so a bad starting placement
    // or a transient first read cannot terminate the run immediately.
    constexpr float kMinimumTravelBeforeCollapseCheckM = 0.01f;
    // [Medium] Post-capture stationary hold before reporting success so the stored endpoint corresponds to a settled
    // robot state.
    constexpr uint16_t kPostCaptureSettleMs = 300U;
    // [Low] EEPROM address used for the persisted front-wall curve. Keep this at zero until other persistent data
    // needs to coexist in the same Teensy EEPROM region.
    constexpr int kStorageAddress = 0;
}

namespace FrontWallCharacterizationConfig = MazeMap::FrontWallCharacterizationConfig;
