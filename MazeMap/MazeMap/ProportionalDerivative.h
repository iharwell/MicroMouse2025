#pragma once

#include "Defines.h"

#include <limits>

namespace MazeMap
{
    // Owns one lean proportional-derivative setup for one specific control objective and one
    // specific signal pairing. This is a small infrastructure owner for gains plus the minimal
    // derivative history needed when callers want the `D` term estimated from sampled error.
    //
    // This type intentionally stays narrow. It owns:
    // - proportional gain,
    // - derivative gain,
    // - one previous-error sample for sampled-derivative evaluation.
    //
    // It intentionally does not own:
    // - integral action,
    // - clamping or saturation,
    // - filtering,
    // - feedforward composition,
    // - angle wrapping or any other signal-specific preprocessing.
    //
    // The same object may be used either as:
    // - a named compile-time setup, or
    // - a mutable runtime setup.
    //
    // Sign conventions:
    // `Compute(error, errorRate)` applies `kp * error + kd * errorRate`.
    // `ComputeFromMeasurementRate(error, measurementRate)` applies
    // `kp * error - kd * measurementRate`.
    class EXPORT ProportionalDerivative final
    {
    public:
        // Builds a setup with zero gains and a cleared sampled-derivative history.
        constexpr ProportionalDerivative() noexcept = default;

        // Builds a setup with the supplied gains and a cleared sampled-derivative history.
        //
        // Parameters:
        // `proportionalGain`:
        // Raw proportional gain stored exactly as provided.
        //
        // `derivativeGain`:
        // Raw derivative gain stored exactly as provided.
        //
        // Behavior:
        // This constructor does not clamp, normalize, or validate the gains. Call `IsValid()` when a
        // caller needs to confirm the stored setup is usable.
        constexpr ProportionalDerivative(
            const float proportionalGain,
            const float derivativeGain) noexcept
            : _proportionalGain(proportionalGain)
            , _derivativeGain(derivativeGain)
            , _previousErrorForDerivative(std::numeric_limits<float>::quiet_NaN())
        {
        }

        // Returns the proportional gain by value.
        float GetProportionalGain() noexcept;

        // Returns the proportional gain by value. This overload is constexpr-safe so named setups
        // may participate in compile-time checks.
        constexpr float GetProportionalGain() const noexcept { return _proportionalGain; }

        // Stores a new proportional gain.
        //
        // Parameters:
        // `proportionalGain`:
        // Raw proportional gain stored exactly as provided.
        //
        // Behavior:
        // This does not normalize the gain, validate it, or reset sampled-derivative history.
        void SetProportionalGain(float proportionalGain) noexcept;

        // Returns the derivative gain by value.
        float GetDerivativeGain() noexcept;

        // Returns the derivative gain by value. This overload is constexpr-safe so named setups
        // may participate in compile-time checks.
        constexpr float GetDerivativeGain() const noexcept { return _derivativeGain; }

        // Stores a new derivative gain.
        //
        // Parameters:
        // `derivativeGain`:
        // Raw derivative gain stored exactly as provided.
        //
        // Behavior:
        // This does not normalize the gain, validate it, or reset sampled-derivative history.
        void SetDerivativeGain(float derivativeGain) noexcept;

        // Replaces both gains at once.
        //
        // Parameters:
        // `proportionalGain`:
        // Raw proportional gain stored exactly as provided.
        //
        // `derivativeGain`:
        // Raw derivative gain stored exactly as provided.
        //
        // Behavior:
        // This does not normalize the gains, validate them, or reset sampled-derivative history.
        void SetGains(float proportionalGain, float derivativeGain) noexcept;

        // Clears the sampled-derivative history. Call this when the current error is known to be
        // discontinuous with the previous sample so the next sampled-error evaluation starts fresh.
        // This is also the explicit reset point after a caller changes gains and wants the next
        // sampled-error derivative estimate to ignore prior error continuity.
        void ResetDerivativeHistory() noexcept;

        // `IsValid()`:
        // Reports whether both gains are finite and non-negative.
        //
        // Behavior:
        // Storage is intentionally permissive: constructors, setters, and mutable getters accept raw
        // values. `IsValid()` is the explicit check for "is this setup usable as a PD law right now?"
        //
        // The non-const overload exists for mutable call sites that already hold a non-const setup.
        bool IsValid() noexcept;
        bool IsValid() const noexcept;

        // Evaluates the PD law when the caller already has `d(error)/dt`.
        //
        // Parameters:
        // `error`:
        // Present control error in the caller's chosen signal units.
        //
        // `errorRate`:
        // Present time derivative of that same error signal.
        //
        // Behavior:
        // Returns `0` when the setup is invalid or either input is non-finite.
        float Compute(float error, float errorRate) noexcept;

        // Evaluates the PD law when the caller already has `d(error)/dt`.
        //
        // Parameters:
        // `error`:
        // Present control error in the caller's chosen signal units.
        //
        // `errorRate`:
        // Present time derivative of that same error signal.
        //
        // Behavior:
        // Returns `0` when the setup is invalid or either input is non-finite.
        float Compute(float error, float errorRate) const noexcept;

        // Evaluates the PD law by estimating `d(error)/dt` from the setup's owned previous-error
        // sample instead of requiring the caller to carry derivative state separately.
        //
        // Parameters:
        // `error`:
        // Present control error in the caller's chosen signal units.
        //
        // `dtSeconds`:
        // Elapsed time since the prior error sample.
        //
        // Behavior:
        // Invalid gains or non-finite inputs clear the owned derivative history and return `0`.
        // The first valid sample after a clear returns only the proportional term and primes the
        // owned previous-error sample. A non-positive or too-small `dtSeconds` also returns only the
        // proportional term after updating the owned previous-error sample.
        float ComputeFromErrorSample(float error, float dtSeconds) noexcept;

        // Evaluates the PD law when the caller has the measured rate of the controlled quantity
        // rather than the derivative of the error itself.
        //
        // Parameters:
        // `error`:
        // Present control error in the caller's chosen signal units.
        //
        // `measurementRate`:
        // Present time derivative of the measured quantity whose rise should oppose the correction.
        //
        // Behavior:
        // Returns `0` when the setup is invalid or either input is non-finite.
        float ComputeFromMeasurementRate(float error, float measurementRate) noexcept;

        // Evaluates the PD law when the caller has the measured rate of the controlled quantity
        // rather than the derivative of the error itself.
        //
        // Parameters:
        // `error`:
        // Present control error in the caller's chosen signal units.
        //
        // `measurementRate`:
        // Present time derivative of the measured quantity whose rise should oppose the correction.
        //
        // Behavior:
        // Returns `0` when the setup is invalid or either input is non-finite.
        float ComputeFromMeasurementRate(float error, float measurementRate) const noexcept;

    private:
        float _proportionalGain = 0.0f;
        float _derivativeGain = 0.0f;
        float _previousErrorForDerivative = std::numeric_limits<float>::quiet_NaN();
    };
}
