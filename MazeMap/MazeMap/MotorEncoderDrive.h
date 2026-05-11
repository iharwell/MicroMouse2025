#pragma once

#include "Defines.h"

#include <cmath>
#include <cstdint>

namespace MazeMap
{
    class Vehicle;

    class MotorEncoderDrive
    {
    private:
        friend class Vehicle;

        static constexpr unsigned long kMinEncoderVelocitySampleMicros = 250UL;
        float _resistance = 1.0f;
        float _voltage = 0.0f;
        float _torqueConstant = 0.0f;
        float _speedConstant = 1.0f;
        float _noLoadCurrent = 0.0f;
        float _currentLimit = 0.0f;
        float _gearRatio = 1.0f;
        float _drivetrainEfficiency = 1.0f;
        float _wheelDiameter = 0.0f;
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

        static float ClampUnit(float value) noexcept
        {
            if (value < -1.0f)
            {
                return -1.0f;
            }
            else if (value > 1.0f)
            {
                return 1.0f;
            }

            return value;
        }

        static int32_t RoundToInt32(float value) noexcept
        {
            return static_cast<int32_t>((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
        }

        static float ClampSymmetric(float value, float limit) noexcept
        {
            if (limit <= 0.0f)
            {
                return 0.0f;
            }

            if (value > limit)
            {
                return limit;
            }

            if (value < -limit)
            {
                return -limit;
            }

            return value;
        }

        static float Signf(float value) noexcept
        {
            if (value > 0.0f)
            {
                return 1.0f;
            }

            if (value < 0.0f)
            {
                return -1.0f;
            }

            return 0.0f;
        }

        static float SignedDirection(float preferredValue, float fallbackValue) noexcept
        {
            constexpr float kSignEpsilon = 1.0e-6f;
            const int preferredSign = (preferredValue > kSignEpsilon) - (preferredValue < -kSignEpsilon);
            if (preferredSign != 0)
            {
                return static_cast<float>(preferredSign);
            }

            const int fallbackSign = (fallbackValue > kSignEpsilon) - (fallbackValue < -kSignEpsilon);
            return static_cast<float>(fallbackSign);
        }

        static uint16_t DriveCommandToPwmCode(float driveCommand) noexcept
        {
            const float magnitude = MazeMap::Math::Absf(ClampUnit(driveCommand));
            return static_cast<uint16_t>(magnitude * static_cast<float>(Platform::kMotorPwmMaxCode) + 0.5f);
        }

        bool hasMotorPins() const noexcept
        {
            return Platform::IsAssignedPin(_motorOutPinA) && Platform::IsAssignedPin(_motorOutPinB);
        }

        bool hasEncoder() const noexcept
        {
            return Platform::IsAssignedEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB);
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
            if (!hasEncoder())
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
            float currentLimit = 0.0f,
            float drivetrainEfficiency = 1.0f) noexcept
            : _resistance(resistance)
            , _voltage(voltage)
            , _torqueConstant(torqueConstant)
            , _speedConstant(speedConstant)
            , _noLoadCurrent(noLoadCurrent)
            , _currentLimit(currentLimit)
            , _gearRatio(gearRatio)
            , _drivetrainEfficiency(drivetrainEfficiency)
            , _wheelDiameter(wheelDiameter)
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

            if (hasMotorPins())
            {
                Platform::ConfigureMotorPwmPin(_motorOutPinA);
                Platform::ConfigureMotorPwmPin(_motorOutPinB);
                coast();
            }

            if (hasEncoder())
            {
                ok = Platform::ConfigureEncoder(_encoderChannel, _encoderInPinA, _encoderInPinB);
                if (ok)
                {
                    setEncoderDistanceMeters(0.0f);
                }
            }

            resetEncoderVelocityEstimate();
            return ok;
        }

    private:
        float getResistance() const noexcept { return _resistance; }
        float getResistance() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getResistance(); }
        void setResistance(float value) noexcept { _resistance = value; }

        float getVoltage() const noexcept { return _voltage; }
        float getVoltage() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getVoltage(); }
        void setVoltage(float value) noexcept { _voltage = value; }

        float getTorqueConstant() const noexcept { return _torqueConstant; }
        float getTorqueConstant() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getTorqueConstant(); }
        void setTorqueConstant(float value) noexcept { _torqueConstant = value; }

        float getSpeedConstant() const noexcept { return _speedConstant; }
        float getSpeedConstant() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getSpeedConstant(); }
        void setSpeedConstant(float value) noexcept { _speedConstant = value; }

        float getNoLoadCurrent() const noexcept { return _noLoadCurrent; }
        float getNoLoadCurrent() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getNoLoadCurrent(); }
        void setNoLoadCurrent(float value) noexcept { _noLoadCurrent = value; }

        float getGearRatio() const noexcept { return _gearRatio; }
        float getGearRatio() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getGearRatio(); }
        void setGearRatio(float value) noexcept { _gearRatio = value; }

        float getWheelDiameter() const noexcept { return _wheelDiameter; }
        float getWheelDiameter() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getWheelDiameter(); }
        void setWheelDiameter(float value) noexcept { _wheelDiameter = value; }

        float getWheelRadius() const noexcept { return 0.5f * _wheelDiameter; }
        float getWheelRadius() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getWheelRadius(); }
        float getWheelCircumference() const noexcept { return PI_F * _wheelDiameter; }
        float getWheelCircumference() noexcept { return const_cast<const MotorEncoderDrive*>(this)->getWheelCircumference(); }

        uint16_t getPulsesPerRev() const noexcept { return _pulsesPerRev; }
        void setPulsesPerRev(uint16_t value) noexcept { _pulsesPerRev = value; }

        uint8_t getMotorOutPinA() const noexcept { return _motorOutPinA; }
        void setMotorOutPinA(uint8_t value) noexcept { _motorOutPinA = value; }

        uint8_t getMotorOutPinB() const noexcept { return _motorOutPinB; }
        void setMotorOutPinB(uint8_t value) noexcept { _motorOutPinB = value; }

        uint8_t getEncoderInPinA() const noexcept { return _encoderInPinA; }
        void setEncoderInPinA(uint8_t value) noexcept
        {
            _encoderInPinA = value;
            resetEncoderVelocityEstimate();
        }

        uint8_t getEncoderInPinB() const noexcept { return _encoderInPinB; }
        void setEncoderInPinB(uint8_t value) noexcept
        {
            _encoderInPinB = value;
            resetEncoderVelocityEstimate();
        }

        uint8_t getEncoderChannel() const noexcept { return _encoderChannel; }
        void setEncoderChannel(uint8_t value) noexcept
        {
            _encoderChannel = value;
            resetEncoderVelocityEstimate();
        }

        bool getInvertMotorDirection() const noexcept { return _invertMotorDirection; }
        void setInvertMotorDirection(bool value) noexcept { _invertMotorDirection = value; }

        bool getInvertEncoderDirection() const noexcept { return _invertEncoderDirection; }
        void setInvertEncoderDirection(bool value) noexcept
        {
            _invertEncoderDirection = value;
            resetEncoderVelocityEstimate();
        }

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

        int32_t distanceToPulses(float distanceMeters) const noexcept
        {
            const float distancePerPulse = getDistancePerPulse();
            assert((distancePerPulse > 0.0f));

            return RoundToInt32(distanceMeters / distancePerPulse);
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
            float noLoadDirection = Signf(groundForceNewtons);
            if (noLoadDirection == 0.0f)
            {
                noLoadDirection = Signf(groundSpeedMetersPerSecond);
            }
            motorCurrent += noLoadDirection * _noLoadCurrent;
            const float motorAngularVelocity = getMotorAngularVelocityAtGroundSpeed(groundSpeedMetersPerSecond);
            return (motorCurrent * _resistance) + (motorAngularVelocity / _speedConstant);
        }

        float getDriveCommandForGroundForce(float groundForceNewtons, float groundSpeedMetersPerSecond) const noexcept
        {
            assert(_voltage > 0.0f);

            return ClampUnit(getMotorVoltageForGroundForce(groundForceNewtons, groundSpeedMetersPerSecond) / _voltage);
        }

        float getMaxForceAtVelocity(float velocityMetersPerSecond) const noexcept
        {
            const float groundSpeed = MazeMap::Math::Absf(velocityMetersPerSecond);
            assert((_wheelDiameter > 0.0f) && (_gearRatio > 0.0f) && (_resistance > 0.0f) && (_speedConstant > 0.0f) && (_torqueConstant > 0.0f) && (_voltage > 0.0f));

            const float wheelRadius = getWheelRadius();
            const float motorAngularVelocity = (groundSpeed / wheelRadius) * _gearRatio;
            const float motorCurrent = ((_voltage - (motorAngularVelocity / _speedConstant)) / _resistance) - _noLoadCurrent;
            if (motorCurrent <= 0.0f)
            {
                return 0.0f;
            }

            return (_torqueConstant * motorCurrent * _gearRatio) / wheelRadius;
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

            const float appliedVoltageV = ClampUnit(driveCommand) * resolvedBatteryVoltageV;
            const float wheelSpeedToBackEmfVoltPerRadps = _gearRatio / _speedConstant;
            const float armatureCurrentFromVoltageA = appliedVoltageV / _resistance;
            const float armatureCurrentFromBackEmfA =
                (wheelBankSpeedRadps * wheelSpeedToBackEmfVoltPerRadps) / _resistance;
            float armatureCurrentA = armatureCurrentFromVoltageA - armatureCurrentFromBackEmfA;

            if (_currentLimit > 0.0f)
            {
                armatureCurrentA = ClampSymmetric(armatureCurrentA, _currentLimit);
            }

            const float noLoadDirection = SignedDirection(armatureCurrentA, wheelBankSpeedRadps);
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
            const float noLoadDirection = SignedDirection(motorTorqueNm, wheelBankSpeedRadps);
            float armatureCurrentA =
                (motorTorqueNm / _torqueConstant) +
                (noLoadDirection * _noLoadCurrent);
            if (_currentLimit > 0.0f)
            {
                armatureCurrentA = ClampSymmetric(armatureCurrentA, _currentLimit);
            }

            const float wheelSpeedToBackEmfVoltPerRadps = _gearRatio / _speedConstant;
            const float backEmfVoltageV = wheelBankSpeedRadps * wheelSpeedToBackEmfVoltPerRadps;
            const float appliedVoltageV = (armatureCurrentA * _resistance) + backEmfVoltageV;
            return ClampUnit(appliedVoltageV / resolvedBatteryVoltageV);
        }

    private:
        void coast() noexcept
        {
            _lastDriveCommand = 0.0f;
            assert(hasMotorPins());

            Platform::DrivePinLow(_motorOutPinA);
            Platform::DrivePinLow(_motorOutPinB);
        }

        void brake() noexcept
        {
            _lastDriveCommand = 0.0f;
            assert(hasMotorPins());

            Platform::DrivePinHigh(_motorOutPinA);
            Platform::DrivePinHigh(_motorOutPinB);
        }

        void setDriveCommand(float driveCommand) noexcept
        {
            _lastDriveCommand = ClampUnit(driveCommand);

            assert(hasMotorPins());

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
            const float maxForce = getMaxForceAtVelocity(groundSpeedMetersPerSecond);
            _lastGroundForceCommand = ClampSymmetric(groundForceNewtons, maxForce);
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
            assert(hasEncoder());

            const int32_t rawCount = Platform::ReadEncoderCount(_encoderChannel);
            return _invertEncoderDirection ? -rawCount : rawCount;
        }

        int32_t consumeEncoderCount() noexcept
        {
            const int32_t count = getEncoderCount();
            // The estimator consumes per-control-cycle encoder deltas. Do not remove this
            // reset or the next cycle will replay cumulative counts and corrupt odometry.
            setEncoderCount(0);
            return count;
        }

        void setEncoderCount(int32_t count) noexcept
        {
            assert(hasEncoder());

            Platform::WriteEncoderCount(_encoderChannel, _invertEncoderDirection ? -count : count);
            resetEncoderVelocityEstimate();
        }

        void resetEncoderCount() noexcept
        {
            setEncoderCount(0);
        }

        float getEncoderDistanceMeters() const noexcept
        {
            return pulsesToDistance(getEncoderCount());
        }

        void setEncoderDistanceMeters(float distanceMeters) noexcept
        {
            setEncoderCount(distanceToPulses(distanceMeters));
        }

        void resetEncoderDistanceMeters(float distanceMeters = 0.0f) noexcept
        {
            setEncoderDistanceMeters(distanceMeters);
        }

        float getEncoderVelocityMetersPerSecond() const noexcept
        {
            updateEncoderVelocityEstimate();
            return _encoderVelocityMetersPerSecond;
        }
    };
}
