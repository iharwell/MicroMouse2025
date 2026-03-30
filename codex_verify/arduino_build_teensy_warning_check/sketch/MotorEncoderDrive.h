#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MotorEncoderDrive.h"
#pragma once

#include "Defines.h"
#include "MotorModelUnits.h"

namespace MazeMap
{
    struct MotorEncoderDrivePhysicalModel
    {
        float nominalVoltageV;
        float nominalNoLoadSpeedRpm;
        float supplyVoltageV;
        float resistanceOhms;
        float torqueConstantNmPerA;
        float noLoadCurrentA;
        float speedConstantRadpsPerVolt;
        float gearRatio;
        float wheelDiameterM;
        uint16_t pulsesPerRev;
    };

    struct MotorEncoderDriveHardwareConfig
    {
        uint8_t motorOutPinA = Platform::kInvalidPin;
        uint8_t motorOutPinB = Platform::kInvalidPin;
        uint8_t encoderInPinA = Platform::kInvalidPin;
        uint8_t encoderInPinB = Platform::kInvalidPin;
        uint8_t encoderChannel = Platform::kInvalidEncoderChannel;
        bool invertMotorDirection = false;
        bool invertEncoderDirection = false;
    };

    class MotorEncoderDrive
    {
    private:
        inline static constexpr MotorEncoderDrivePhysicalModel kSharedPhysicalModel = {
            6.0f,
            14100.0f,
            8.4f,
            4.31f,
            MilliNewtonMetersToNewtonMeters(3.96f),
            MilliAmpsToAmps(45.9f),
            ComputeMotorSpeedConstantRadpsPerVolt(
                14100.0f,
                6.0f,
                MilliAmpsToAmps(45.9f),
                4.31f),
            56.0f / 17.0f,
            // March 22, 2026 low-speed straight-audit fit: aux001-aux003 speed_idx 0 still over-reported outbound
            // encoder distance by about 3.05 mm on the 0.72 m north-corridor run, so trim the shared wheel diameter
            // down by 0.42% to keep the fixed-distance phases from finishing short.
            0.025220f,
            4096U
        };
        inline static constexpr MotorEncoderDriveHardwareConfig kLeftHardwareConfig = {
            24U,
            25U,
            2U,
            3U,
            2U,
            true,
            false
        };
        inline static constexpr MotorEncoderDriveHardwareConfig kRightHardwareConfig = {
            5U,
            6U,
            7U,
            8U,
            1U,
            true,
            false
        };
        static constexpr unsigned long kMinEncoderVelocitySampleMicros = 250UL;
        float _resistance = 1.0f;
        float _voltage = 0.0f;
        float _torqueConstant = 0.0f;
        float _speedConstant = 1.0f;
        float _noLoadCurrent = 0.0f;
        float _gearRatio = 1.0f;
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

            if (value > 1.0f)
            {
                return 1.0f;
            }

            return value;
        }

        static float Absf(float value) noexcept
        {
            return (value < 0.0f) ? -value : value;
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

        static uint16_t DriveCommandToPwmCode(float driveCommand) noexcept
        {
            const float magnitude = Absf(ClampUnit(driveCommand));
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

    public:
        MotorEncoderDrive() = default;

        static constexpr const MotorEncoderDrivePhysicalModel& GetSharedPhysicalModel() noexcept
        {
            return kSharedPhysicalModel;
        }

        static constexpr const MotorEncoderDriveHardwareConfig& GetLeftHardwareConfig() noexcept
        {
            return kLeftHardwareConfig;
        }

        static constexpr const MotorEncoderDriveHardwareConfig& GetRightHardwareConfig() noexcept
        {
            return kRightHardwareConfig;
        }

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
            bool invertEncoderDirection = false) noexcept
            : _resistance(resistance)
            , _voltage(voltage)
            , _torqueConstant(torqueConstant)
            , _speedConstant(speedConstant)
            , _noLoadCurrent(noLoadCurrent)
            , _gearRatio(gearRatio)
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

        MotorEncoderDrive(
            const MotorEncoderDrivePhysicalModel& physicalModel,
            const MotorEncoderDriveHardwareConfig& hardwareConfig) noexcept
            : MotorEncoderDrive(
                physicalModel.resistanceOhms,
                physicalModel.supplyVoltageV,
                physicalModel.torqueConstantNmPerA,
                physicalModel.speedConstantRadpsPerVolt,
                physicalModel.noLoadCurrentA,
                physicalModel.gearRatio,
                physicalModel.wheelDiameterM,
                physicalModel.pulsesPerRev,
                hardwareConfig.motorOutPinA,
                hardwareConfig.motorOutPinB,
                hardwareConfig.encoderInPinA,
                hardwareConfig.encoderInPinB,
                hardwareConfig.encoderChannel,
                hardwareConfig.invertMotorDirection,
                hardwareConfig.invertEncoderDirection)
        {
        }

        static MotorEncoderDrive CreateDefaultLeftDrive() noexcept
        {
            return MotorEncoderDrive(GetSharedPhysicalModel(), GetLeftHardwareConfig());
        }

        static MotorEncoderDrive CreateDefaultRightDrive() noexcept
        {
            return MotorEncoderDrive(GetSharedPhysicalModel(), GetRightHardwareConfig());
        }

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

        float getResistance() const noexcept { return _resistance; }
        void setResistance(float value) noexcept { _resistance = value; }

        float getVoltage() const noexcept { return _voltage; }
        void setVoltage(float value) noexcept { _voltage = value; }

        float getTorqueConstant() const noexcept { return _torqueConstant; }
        void setTorqueConstant(float value) noexcept { _torqueConstant = value; }

        float getSpeedConstant() const noexcept { return _speedConstant; }
        void setSpeedConstant(float value) noexcept { _speedConstant = value; }

        float getNoLoadCurrent() const noexcept { return _noLoadCurrent; }
        void setNoLoadCurrent(float value) noexcept { _noLoadCurrent = value; }

        float getGearRatio() const noexcept { return _gearRatio; }
        void setGearRatio(float value) noexcept { _gearRatio = value; }

        float getWheelDiameter() const noexcept { return _wheelDiameter; }
        void setWheelDiameter(float value) noexcept { _wheelDiameter = value; }

        float getWheelRadius() const noexcept { return 0.5f * _wheelDiameter; }
        float getWheelCircumference() const noexcept { return PI_F * _wheelDiameter; }

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

        float getDistancePerPulse() const noexcept
        {
            if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || (_pulsesPerRev == 0U))
            {
                return 0.0f;
            }

            return getWheelCircumference() / (_gearRatio * static_cast<float>(_pulsesPerRev));
        }

        float pulsesToDistance(int32_t pulses) const noexcept
        {
            return static_cast<float>(pulses) * getDistancePerPulse();
        }

        int32_t distanceToPulses(float distanceMeters) const noexcept
        {
            const float distancePerPulse = getDistancePerPulse();
            if (!(distancePerPulse > 0.0f))
            {
                return 0;
            }

            return RoundToInt32(distanceMeters / distancePerPulse);
        }

        float getMotorAngularVelocityAtGroundSpeed(float groundSpeedMetersPerSecond) const noexcept
        {
            const float wheelRadius = getWheelRadius();
            if (!(wheelRadius > 0.0f) || !(_gearRatio > 0.0f))
            {
                return 0.0f;
            }

            return (groundSpeedMetersPerSecond / wheelRadius) * _gearRatio;
        }

        float getGroundSpeedFromMotorAngularVelocity(float motorAngularVelocity) const noexcept
        {
            if (!(_gearRatio > 0.0f))
            {
                return 0.0f;
            }

            return (motorAngularVelocity / _gearRatio) * getWheelRadius();
        }

        float getMotorCurrentForGroundForce(float groundForceNewtons) const noexcept
        {
            if (!(_torqueConstant > 0.0f) || !(_gearRatio > 0.0f) || !(_wheelDiameter > 0.0f))
            {
                return 0.0f;
            }

            return (groundForceNewtons * getWheelRadius()) / (_torqueConstant * _gearRatio);
        }

        float getMotorVoltageForGroundForce(float groundForceNewtons, float groundSpeedMetersPerSecond) const noexcept
        {
            if (!(_resistance > 0.0f) || !(_speedConstant > 0.0f) || !(_torqueConstant > 0.0f) || !(_gearRatio > 0.0f) || !(_wheelDiameter > 0.0f))
            {
                return 0.0f;
            }

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
            if (!(_voltage > 0.0f))
            {
                return 0.0f;
            }

            return ClampUnit(getMotorVoltageForGroundForce(groundForceNewtons, groundSpeedMetersPerSecond) / _voltage);
        }

        float getMaxForceAtVelocity(float velocityMetersPerSecond) const noexcept
        {
            const float groundSpeed = Absf(velocityMetersPerSecond);
            if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || !(_resistance > 0.0f) || !(_speedConstant > 0.0f) || !(_torqueConstant > 0.0f) || !(_voltage > 0.0f))
            {
                return 0.0f;
            }

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
            const float limitedForce = Absf(maxTractiveForce);
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

        void coast() noexcept
        {
            _lastDriveCommand = 0.0f;
            if (!hasMotorPins())
            {
                return;
            }

            Platform::DrivePinLow(_motorOutPinA);
            Platform::DrivePinLow(_motorOutPinB);
        }

        void brake() noexcept
        {
            _lastDriveCommand = 0.0f;
            if (!hasMotorPins())
            {
                return;
            }

            Platform::DrivePinHigh(_motorOutPinA);
            Platform::DrivePinHigh(_motorOutPinB);
        }

        void setDriveCommand(float driveCommand) noexcept
        {
            _lastDriveCommand = ClampUnit(driveCommand);

            if (!hasMotorPins())
            {
                return;
            }

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
            if (!hasEncoder())
            {
                return 0;
            }

            const int32_t rawCount = Platform::ReadEncoderCount(_encoderChannel);
            return _invertEncoderDirection ? -rawCount : rawCount;
        }

        void setEncoderCount(int32_t count) noexcept
        {
            if (!hasEncoder())
            {
                return;
            }

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
