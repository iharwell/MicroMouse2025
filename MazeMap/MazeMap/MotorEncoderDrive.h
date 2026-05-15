#pragma once

#include "Defines.h"

#include <cmath>
#include <cstdint>

namespace MazeMap
{
    class PlantModel;
    class Vehicle;

    class MotorEncoderDrive
    {
    private:
        friend class PlantModel;
        friend class Vehicle;

        static constexpr unsigned long kMinEncoderVelocitySampleMicros = 250UL;
        static constexpr float kDefaultWheelBankEquivalentInertiaKgM2 = 240.0e-9f;
        // Drive-owned tire parameters. Keep these here because they describe the motor/encoder wheel bank:
        // 25.0 mm wheel OD, 17.2 mm hub, solid 20A rubber, and two wheels per bank.
        // The measured contact span gives one tire contact width:
        // (78.68 mm outside span - 70.04 mm inside span) / 2 = 4.32 mm.
        // Measured tire stiffness from solid-rubber contact data:
        // normal per tire = (0.138 kg * 9.80665 m/s^2 + 0.56 N fan downforce) / 4 = 0.4783 N,
        // contact length ~= 2.71 mm, K_tire ~= 164.6 N/m, K_bank ~= 329.2 N/m.
        // That maps to C_alpha ~= 0.223 N/rad per tire and C_kappa ~= 2.06 N per tire.
        // The plant stores longitudinal slip stiffness per two-wheel bank and cornering stiffness per tire.
        static constexpr float kDefaultLongitudinalTireStiffnessN = 4.12f;
        static constexpr float kDefaultCorneringStiffnessNPerRad = 0.223f;
        float _resistance = 1.0f;
        float _voltage = 0.0f;
        float _torqueConstant = 0.0f;
        float _speedConstant = 1.0f;
        float _noLoadCurrent = 0.0f;
        float _gearRatio = 1.0f;
        float _drivetrainEfficiency = 1.0f;
        float _wheelDiameter = 0.0f;
        float _equivalentWheelInertiaKgM2 = 0.0f;
        float _longitudinalTireStiffnessN = 0.0f;
        float _corneringStiffnessNPerRad = 0.0f;
        uint16_t _pulsesPerRev = 1U;
        uint8_t _motorOutPinA = Platform::kInvalidPin;
        uint8_t _motorOutPinB = Platform::kInvalidPin;
        uint8_t _encoderInPinA = Platform::kInvalidPin;
        uint8_t _encoderInPinB = Platform::kInvalidPin;
        uint8_t _encoderChannel = Platform::kInvalidEncoderChannel;
        bool _invertMotorDirection = false;
        bool _invertEncoderDirection = false;
        float _lastDriveCommand = 0.0f;
        float _lastGroundForceCommand = 0.0f;
        mutable bool _encoderVelocityInitialized = false;
        mutable int32_t _lastEncoderCountSample = 0;
        mutable unsigned long _lastEncoderSampleMicros = 0UL;
        mutable float _encoderVelocityMetersPerSecond = 0.0f;

        static uint16_t DriveCommandToPwmCode(float driveCommand) noexcept
        {
            const float magnitude = MazeMap::Math::Absf(driveCommand);
            return static_cast<uint16_t>(magnitude * static_cast<float>(Platform::kMotorPwmMaxCode) + 0.5f);
        }

        void resetEncoderVelocityEstimate() noexcept
        {
            _encoderVelocityInitialized = false;
            _lastEncoderCountSample = 0;
            _lastEncoderSampleMicros = 0UL;
            _encoderVelocityMetersPerSecond = 0.0f;
        }

        void updateEncoderVelocityEstimate() const noexcept
        {
            if (!Platform::IsAssignedEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB))
            {
                _encoderVelocityMetersPerSecond = 0.0f;
                return;
            }

            const unsigned long nowMicros = micros();
            const int32_t countNow = getEncoderCount();

            if (!_encoderVelocityInitialized)
            {
                _encoderVelocityInitialized = true;
                _lastEncoderCountSample = countNow;
                _lastEncoderSampleMicros = nowMicros;
                _encoderVelocityMetersPerSecond = 0.0f;
                return;
            }

            const unsigned long elapsedMicros = nowMicros - _lastEncoderSampleMicros;

            // Multiple reads can happen inside one control cycle. Reuse the last valid
            // estimate until enough time has elapsed to avoid one-count speed spikes.
            if (elapsedMicros < kMinEncoderVelocitySampleMicros)
            {
                return;
            }

            const int32_t deltaCounts = countNow - _lastEncoderCountSample;
            _encoderVelocityMetersPerSecond = pulsesToDistance(deltaCounts) / (static_cast<float>(elapsedMicros) * 1.0e-6f);
            _lastEncoderCountSample = countNow;
            _lastEncoderSampleMicros = nowMicros;
        }

    private:
        MotorEncoderDrive() = default;

        MotorEncoderDrive(
            float resistance,
            float voltage,
            float torqueConstant,
            float speedConstant,
            float noLoadCurrent,
            float gearRatio,
            float wheelDiameter,
            uint16_t pulsesPerRev,
            uint8_t motorOutPinA,
            uint8_t motorOutPinB,
            uint8_t encoderInPinA,
            uint8_t encoderInPinB,
            uint8_t encoderChannel = Platform::kInvalidEncoderChannel,
            bool invertMotorDirection = false,
            bool invertEncoderDirection = false,
            float drivetrainEfficiency = 1.0f,
            float equivalentWheelInertiaKgM2 = kDefaultWheelBankEquivalentInertiaKgM2,
            float longitudinalTireStiffnessN = kDefaultLongitudinalTireStiffnessN,
            float corneringStiffnessNPerRad = kDefaultCorneringStiffnessNPerRad) noexcept
            : _resistance(resistance)
            , _voltage(voltage)
            , _torqueConstant(torqueConstant)
            , _speedConstant(speedConstant)
            , _noLoadCurrent(noLoadCurrent)
            , _gearRatio(gearRatio)
            , _drivetrainEfficiency(drivetrainEfficiency)
            , _wheelDiameter(wheelDiameter)
            , _equivalentWheelInertiaKgM2(equivalentWheelInertiaKgM2)
            , _longitudinalTireStiffnessN(longitudinalTireStiffnessN)
            , _corneringStiffnessNPerRad(corneringStiffnessNPerRad)
            , _pulsesPerRev(pulsesPerRev)
            , _motorOutPinA(motorOutPinA)
            , _motorOutPinB(motorOutPinB)
            , _encoderInPinA(encoderInPinA)
            , _encoderInPinB(encoderInPinB)
            , _encoderChannel(encoderChannel)
            , _invertMotorDirection(invertMotorDirection)
            , _invertEncoderDirection(invertEncoderDirection)
        {
        }

    public:
        bool begin()
        {
            bool ok = true;

            if (Platform::IsAssignedPin(_motorOutPinA) && Platform::IsAssignedPin(_motorOutPinB))
            {
                Platform::ConfigureMotorPwmPin(_motorOutPinA);
                Platform::ConfigureMotorPwmPin(_motorOutPinB);
                coast();
            }

            if (Platform::IsAssignedEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB))
            {
                ok = Platform::ConfigureEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB);
                if (ok)
                {
                    resetEncoderCount();
                }
            }

            resetEncoderVelocityEstimate();
            return ok;
        }

    private:
        float getResistance() const noexcept { return _resistance; }

        float getVoltage() const noexcept { return _voltage; }

        float getTorqueConstant() const noexcept { return _torqueConstant; }

        float getSpeedConstant() const noexcept { return _speedConstant; }

        float getNoLoadCurrent() const noexcept { return _noLoadCurrent; }

        float getGearRatio() const noexcept { return _gearRatio; }

        float getWheelDiameter() const noexcept { return _wheelDiameter; }

        float getWheelRadius() const noexcept { return 0.5f * _wheelDiameter; }
        float getWheelCircumference() const noexcept { return PI_F * _wheelDiameter; }

        float getEquivalentWheelInertiaKgM2() const noexcept { return _equivalentWheelInertiaKgM2; }

        float getLongitudinalTireStiffnessN() const noexcept { return _longitudinalTireStiffnessN; }

        float getCorneringStiffnessNPerRad() const noexcept { return _corneringStiffnessNPerRad; }

        uint16_t getPulsesPerRev() const noexcept { return _pulsesPerRev; }

        uint8_t getMotorOutPinA() const noexcept { return _motorOutPinA; }

        uint8_t getMotorOutPinB() const noexcept { return _motorOutPinB; }

        uint8_t getEncoderInPinA() const noexcept { return _encoderInPinA; }

        uint8_t getEncoderInPinB() const noexcept { return _encoderInPinB; }

        uint8_t getEncoderChannel() const noexcept { return _encoderChannel; }

        bool getInvertMotorDirection() const noexcept { return _invertMotorDirection; }

        bool getInvertEncoderDirection() const noexcept { return _invertEncoderDirection; }

    public:
        float getDistancePerPulse() const noexcept
        {
            assert((_wheelDiameter > 0.0f) && (_gearRatio > 0.0f) && (_pulsesPerRev >= 0U));
            return getWheelCircumference() / (_gearRatio * static_cast<float>(_pulsesPerRev));
        }

        float pulsesToDistance(int32_t pulses) const noexcept
        {
            return static_cast<float>(pulses) * getDistancePerPulse();
        }

        float getMotorAngularVelocityAtGroundSpeed(float groundSpeedMetersPerSecond) const noexcept
        {
            const float wheelRadius = getWheelRadius();
            assert((wheelRadius > 0.0f) && (_gearRatio > 0.0f));

            return (groundSpeedMetersPerSecond / wheelRadius) * _gearRatio;
        }

        float getGroundSpeedFromMotorAngularVelocity(float motorAngularVelocity) const noexcept
        {
            assert(_gearRatio > 0.0f);

            return (motorAngularVelocity / _gearRatio) * getWheelRadius();
        }

        float getMotorCurrentForGroundForce(float groundForceNewtons) const noexcept
        {
            assert((_torqueConstant > 0.0f) && (_gearRatio > 0.0f) && (_wheelDiameter > 0.0f));

            return (groundForceNewtons * getWheelRadius()) / (_torqueConstant * _gearRatio);
        }

        float getMotorVoltageForGroundForce(float groundForceNewtons, float groundSpeedMetersPerSecond) const noexcept
        {
            assert((_resistance > 0.0f) && (_speedConstant > 0.0f) && (_torqueConstant > 0.0f) && (_gearRatio > 0.0f) && (_wheelDiameter > 0.0f));

            float motorCurrent = getMotorCurrentForGroundForce(groundForceNewtons);
            float noLoadDirection = static_cast<float>((groundForceNewtons > 0.0f) - (groundForceNewtons < 0.0f));
            if (noLoadDirection == 0.0f)
            {
                noLoadDirection = static_cast<float>((groundSpeedMetersPerSecond > 0.0f) - (groundSpeedMetersPerSecond < 0.0f));
            }
            motorCurrent += noLoadDirection * _noLoadCurrent;
            const float motorAngularVelocity = getMotorAngularVelocityAtGroundSpeed(groundSpeedMetersPerSecond);
            return (motorCurrent * _resistance) + (motorAngularVelocity / _speedConstant);
        }

        float getDriveCommandForGroundForce(float groundForceNewtons, float groundSpeedMetersPerSecond) const noexcept
        {
            assert(_voltage > 0.0f);

            return getMotorVoltageForGroundForce(groundForceNewtons, groundSpeedMetersPerSecond) / _voltage;
        }

        float getSpeedAtForceLimit(float maxTractiveForce) const noexcept
        {
            const float limitedForce = MazeMap::Math::Absf(maxTractiveForce);
            if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || !(_resistance > 0.0f) || !(_speedConstant > 0.0f) || !(_torqueConstant > 0.0f) || !(_voltage > 0.0f))
            {
                return 0.0f;
            }

            const float wheelRadius = getWheelRadius();
            const float availableCurrent = (_voltage / _resistance) - _noLoadCurrent;
            if (availableCurrent <= 0.0f)
            {
                return 0.0f;
            }

            const float stallForce = (_torqueConstant * availableCurrent * _gearRatio) / wheelRadius;
            if (limitedForce > stallForce)
            {
                return 0.0f;
            }

            const float noLoadSpeed = (wheelRadius * _speedConstant * _voltage) / _gearRatio;
            const float loadCurrent = getMotorCurrentForGroundForce(limitedForce) + _noLoadCurrent;
            const float speed = (wheelRadius * _speedConstant / _gearRatio) * (_voltage - (loadCurrent * _resistance));
            if (speed < 0.0f)
            {
                return 0.0f;
            }

            return (speed > noLoadSpeed) ? noLoadSpeed : speed;
        }

        // Converts a normalized drive command into wheel-bank torque, not
        // motor-shaft torque, for this device at the supplied wheel-bank speed.
        float getTorqueFromCommand(
            float driveCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV = 0.0f) const noexcept
        {
            if (!((_resistance > 0.0f) &&
                (_speedConstant > 0.0f) &&
                (_torqueConstant > 0.0f) &&
                (_gearRatio > 0.0f)))
            {
                return 0.0f;
            }

            const float resolvedBatteryVoltageV =
                (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
                batteryVoltageV :
                _voltage;
            if (!(resolvedBatteryVoltageV > 0.0f))
            {
                return 0.0f;
            }

            const float appliedVoltageV = driveCommand * resolvedBatteryVoltageV;
            const float wheelSpeedToBackEmfVoltPerRadps = _gearRatio / _speedConstant;
            const float armatureCurrentFromVoltageA = appliedVoltageV / _resistance;
            const float armatureCurrentFromBackEmfA =
                (wheelBankSpeedRadps * wheelSpeedToBackEmfVoltPerRadps) / _resistance;
            float armatureCurrentA = armatureCurrentFromVoltageA - armatureCurrentFromBackEmfA;

            constexpr float kSignEpsilon = 1.0e-6f;
            const int armatureDirection =
                (armatureCurrentA > kSignEpsilon) - (armatureCurrentA < -kSignEpsilon);
            const int wheelDirection =
                (wheelBankSpeedRadps > kSignEpsilon) - (wheelBankSpeedRadps < -kSignEpsilon);
            const float noLoadDirection =
                static_cast<float>((armatureDirection != 0) ? armatureDirection : wheelDirection);
            float loadCurrentA = armatureCurrentA - (noLoadDirection * _noLoadCurrent);
            if ((noLoadDirection > 0.0f) && (loadCurrentA < 0.0f))
            {
                loadCurrentA = 0.0f;
            }
            else if ((noLoadDirection < 0.0f) && (loadCurrentA > 0.0f))
            {
                loadCurrentA = 0.0f;
            }

            return _torqueConstant * _gearRatio * _drivetrainEfficiency * loadCurrentA;
        }

        float getForwardForceFromCommand(
            float driveCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV = 0.0f) const noexcept
        {
            const float wheelRadius = getWheelRadius();
            return (wheelRadius > 0.0f) ?
                (getTorqueFromCommand(driveCommand, wheelBankSpeedRadps, batteryVoltageV) / wheelRadius) :
                0.0f;
        }

        float getPeakForwardForceAtBankSpeed(
            float wheelBankSpeedRadps,
            float batteryVoltageV = 0.0f) const noexcept
        {
            return getForwardForceFromCommand(1.0f, wheelBankSpeedRadps, batteryVoltageV);
        }

        // Converts a requested wheel-bank torque, not motor-shaft torque, into the
        // normalized drive command for this device at the supplied wheel-bank speed.
        float getCommandFromTorque(
            float wheelBankTorqueNm,
            float wheelBankSpeedRadps,
            float batteryVoltageV = 0.0f) const noexcept
        {
            if (!(std::isfinite(wheelBankTorqueNm) &&
                std::isfinite(wheelBankSpeedRadps) &&
                (_resistance > 0.0f) &&
                (_speedConstant > 0.0f) &&
                (_torqueConstant > 0.0f) &&
                (_gearRatio > 0.0f) &&
                (_drivetrainEfficiency > 0.0f)))
            {
                return 0.0f;
            }

            const float resolvedBatteryVoltageV =
                (std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ?
                batteryVoltageV :
                _voltage;
            if (!(resolvedBatteryVoltageV > 0.0f))
            {
                return 0.0f;
            }

            const float motorTorqueNm = wheelBankTorqueNm / (_gearRatio * _drivetrainEfficiency);
            constexpr float kSignEpsilon = 1.0e-6f;
            const int torqueDirection =
                (motorTorqueNm > kSignEpsilon) - (motorTorqueNm < -kSignEpsilon);
            const int wheelDirection =
                (wheelBankSpeedRadps > kSignEpsilon) - (wheelBankSpeedRadps < -kSignEpsilon);
            const float noLoadDirection =
                static_cast<float>((torqueDirection != 0) ? torqueDirection : wheelDirection);
            float armatureCurrentA =
                (motorTorqueNm / _torqueConstant) +
                (noLoadDirection * _noLoadCurrent);
            const float wheelSpeedToBackEmfVoltPerRadps = _gearRatio / _speedConstant;
            const float backEmfVoltageV = wheelBankSpeedRadps * wheelSpeedToBackEmfVoltPerRadps;
            const float appliedVoltageV = (armatureCurrentA * _resistance) + backEmfVoltageV;
            return appliedVoltageV / resolvedBatteryVoltageV;
        }

    private:
        void coast() noexcept
        {
            _lastDriveCommand = 0.0f;
            assert(Platform::IsAssignedPin(_motorOutPinA) && Platform::IsAssignedPin(_motorOutPinB));

            Platform::DrivePinLow(_motorOutPinA);
            Platform::DrivePinLow(_motorOutPinB);
        }

        void brake() noexcept
        {
            _lastDriveCommand = 0.0f;
            assert(Platform::IsAssignedPin(_motorOutPinA) && Platform::IsAssignedPin(_motorOutPinB));

            Platform::DrivePinHigh(_motorOutPinA);
            Platform::DrivePinHigh(_motorOutPinB);
        }

        void setDriveCommand(float driveCommand) noexcept
        {
            _lastDriveCommand = driveCommand;

            assert(Platform::IsAssignedPin(_motorOutPinA) && Platform::IsAssignedPin(_motorOutPinB));

            float hardwareCommand = _invertMotorDirection ? -_lastDriveCommand : _lastDriveCommand;
            if (hardwareCommand > 0.0f)
            {
                Platform::DrivePinLow(_motorOutPinB);
                Platform::WriteMotorPwmCode(_motorOutPinA, DriveCommandToPwmCode(hardwareCommand));
                return;
            }

            if (hardwareCommand < 0.0f)
            {
                Platform::DrivePinLow(_motorOutPinA);
                Platform::WriteMotorPwmCode(_motorOutPinB, DriveCommandToPwmCode(hardwareCommand));
                return;
            }

            coast();
        }

        float getDriveCommand() const noexcept
        {
            return _lastDriveCommand;
        }

        void setGroundForce(float groundForceNewtons, float groundSpeedMetersPerSecond) noexcept
        {
            _lastGroundForceCommand = groundForceNewtons;
            setDriveCommand(getDriveCommandForGroundForce(_lastGroundForceCommand, groundSpeedMetersPerSecond));
        }

        void setGroundForce(float groundForceNewtons) noexcept
        {
            setGroundForce(groundForceNewtons, getEncoderVelocityMetersPerSecond());
        }

        float getGroundForceCommand() const noexcept
        {
            return _lastGroundForceCommand;
        }

        int32_t getEncoderCount() const noexcept
        {
            assert(Platform::IsAssignedEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB));

            const int32_t rawCount = Platform::ReadEncoderCount(_encoderChannel);
            return _invertEncoderDirection ? -rawCount : rawCount;
        }

        int32_t consumeEncoderCount() noexcept
        {
            const int32_t count = getEncoderCount();
            // The estimator consumes per-control-cycle encoder deltas. Do not remove this
            // reset or the next cycle will replay cumulative counts and corrupt odometry.
            resetEncoderCount();
            return count;
        }

        void resetEncoderCount() noexcept
        {
            assert(Platform::IsAssignedEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB));

            Platform::WriteEncoderCount(_encoderChannel, 0);
            resetEncoderVelocityEstimate();
        }

        float getEncoderDistanceMeters() const noexcept
        {
            return pulsesToDistance(getEncoderCount());
        }

        float getEncoderVelocityMetersPerSecond() const noexcept
        {
            updateEncoderVelocityEstimate();
            return _encoderVelocityMetersPerSecond;
        }
    };
}
