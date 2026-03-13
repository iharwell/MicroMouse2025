#pragma once
#include "Defines.h"


namespace MazeMap
{
	class EXPORT MotorEncoderDrive
	{
	private:
		float _resistance;
		float _voltage;
		float _torqueConstant;
		float _speedConstant;
		float _gearRatio;
		float _wheelDiameter;
		uint16_t _pulsesPerRev;
		uint8_t _motorOutPinA;
		uint8_t _motorOutPinB;
		uint8_t _encoderInPinA;
		uint8_t _encoderInPinB;

		static constexpr uint16_t kMotorPwmCounts = 1875U;
		static constexpr uint16_t kMotorPwmMaxCode = kMotorPwmCounts - 1U; // 1874
		struct FlexPwmPinInfo
		{
			IMXRT_FLEXPWM_t* module;
			uint8_t submodule;
			uint8_t channel; // 0 = X, 1 = A, 2 = B
		};

		static bool resolveMotorFlexPwmPin(uint8_t pin, FlexPwmPinInfo& info)
		{
			switch (pin)
			{
			case 5:
				info.module = &IMXRT_FLEXPWM2;
				info.submodule = 1;
				info.channel = 1; // A
				return true;

			case 6:
				info.module = &IMXRT_FLEXPWM2;
				info.submodule = 2;
				info.channel = 1; // A
				return true;

			case 24:
				info.module = &IMXRT_FLEXPWM1;
				info.submodule = 2;
				info.channel = 0; // X
				return true;

			case 25:
				info.module = &IMXRT_FLEXPWM1;
				info.submodule = 3;
				info.channel = 0; // X
				return true;

			default:
				return false;
			}
		}

		static void driveLow(uint8_t pin)
		{
			pinMode(pin, OUTPUT);
			digitalWrite(pin, LOW);
		}

		static void driveHigh(uint8_t pin)
		{
			pinMode(pin, OUTPUT);
			digitalWrite(pin, HIGH);
		}

		static void writeExactPwmCode(uint8_t pin, uint16_t code)
		{
			if (code > kMotorPwmMaxCode)
			{
				code = kMotorPwmMaxCode;
			}

			FlexPwmPinInfo info;

			if (!resolveMotorFlexPwmPin(pin, info))
			{
				return;
			}

			// Re-enter PWM mode on this pin. The exact duty update happens below.
			analogWrite(pin, 0U);

			const uint16_t mask = static_cast<uint16_t>(1U << info.submodule);

			info.module->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);

			switch (info.channel)
			{
			case 0: // X
				info.module->SM[info.submodule].VAL0 = kMotorPwmMaxCode - code;
				info.module->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
				break;

			case 1: // A
				info.module->SM[info.submodule].VAL3 = code;
				info.module->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
				break;

			case 2: // B
				info.module->SM[info.submodule].VAL5 = code;
				info.module->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
				break;

			default:
				info.module->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
				return;
			}

			info.module->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
		}
	public:
		// Resistance in Ohms
		inline float    getResistance() { return const_cast<const MotorEncoderDrive*>(this)->getResistance(); }
		// Resistance in Ohms
		inline float    getResistance() const { return _resistance; }
		// Resistance in Ohms
		inline void     setResistance(float v) { _resistance = v; }

		inline float    getVoltage() { return const_cast<const MotorEncoderDrive*>(this)->getVoltage(); }
		inline float    getVoltage() const { return _voltage; }
		inline void     setVoltage(float v) { _voltage = v; }

		// Torque Constant in N*m/A
		inline float    getTorqueConstant() { return const_cast<const MotorEncoderDrive*>(this)->getTorqueConstant(); }
		inline float    getTorqueConstant() const { return _torqueConstant; }
		inline void     setTorqueConstant(float v) { _torqueConstant = v; }

		inline float    getSpeedConstant() { return const_cast<const MotorEncoderDrive*>(this)->getSpeedConstant(); }
		inline float    getSpeedConstant() const { return _speedConstant; }
		inline void     setSpeedConstant(float v) { _speedConstant = v; }

		inline float    getGearRatio() { return const_cast<const MotorEncoderDrive*>(this)->getGearRatio(); }
		inline float    getGearRatio() const { return _gearRatio; }
		inline void     setGearRatio(float v) { _gearRatio = v; }

		inline float    getWheelDiameter() { return const_cast<const MotorEncoderDrive*>(this)->getWheelDiameter(); }
		inline float    getWheelDiameter() const { return _wheelDiameter; }
		inline void     setWheelDiameter(float v) { _wheelDiameter = v; }

		inline uint16_t getPulsesPerRev() { return const_cast<const MotorEncoderDrive*>(this)->getPulsesPerRev(); }
		inline uint16_t getPulsesPerRev() const { return _pulsesPerRev; }
		inline void     setPulsesPerRev(uint16_t v) { _pulsesPerRev = v; }

		inline uint8_t getMotorOutPinA() { return const_cast<const MotorEncoderDrive*>(this)->getMotorOutPinA(); }
		inline uint8_t getMotorOutPinA() const { return _motorOutPinA; }
		inline void setMotorOutPinA(uint8_t v) { _motorOutPinA = v; }

		inline uint8_t getMotorOutPinB() { return const_cast<const MotorEncoderDrive*>(this)->getMotorOutPinB(); }
		inline uint8_t getMotorOutPinB() const { return _motorOutPinB; }
		inline void setMotorOutPinB(uint8_t v) { _motorOutPinB = v; }

		inline uint8_t getEncoderInPinA() { return const_cast<const MotorEncoderDrive*>(this)->getEncoderInPinA(); }
		inline uint8_t getEncoderInPinA() const { return _encoderInPinA; }
		inline void setEncoderInPinA(uint8_t v) { _encoderInPinA = v; }

		inline uint8_t getEncoderInPinB() { return const_cast<const MotorEncoderDrive*>(this)->getEncoderInPinB(); }
		inline uint8_t getEncoderInPinB() const { return _encoderInPinB; }
		inline void setEncoderInPinB(uint8_t v) { _encoderInPinB = v; }

		inline float getDistancePerPulse() { return const_cast<const MotorEncoderDrive*>(this)->getDistancePerPulse(); }
		inline float getDistancePerPulse() const { return (3.14159265358979323846f * _wheelDiameter) / (_gearRatio * static_cast<float>(_pulsesPerRev)); }

		// Assumes: _speedConstant = Kv in (rad/s)/V,
		// _torqueConstant = Kt in N·m/A,
		// _wheelDiameter in meters,
		// velocity in m/s,
		// result in newtons.
		inline float getMaxForceAtVelocity(float velocity) { return const_cast<const MotorEncoderDrive*>(this)->getMaxForceAtVelocity(velocity); }

		inline float getMaxForceAtVelocity(float velocity) const
		{
#ifdef _DEBUG
			const float v = (velocity < 0.0f) ? -velocity : velocity;
			if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || !(_resistance > 0.0f) || !(_speedConstant > 0.0f)) return 0.0f;
#endif
			const float r = 0.5f * _wheelDiameter;
			const float omega_m = (v / r) * _gearRatio;
			const float i = (_voltage - (omega_m / _speedConstant)) / _resistance;
			if (i <= 0.0f) return 0.0f;
			return (_torqueConstant * i * _gearRatio) / r;
		}
		inline float getSpeedAtForceLimit(float maxTractiveForce) const
		{
			const float F = (maxTractiveForce < 0.0f) ? -maxTractiveForce : maxTractiveForce;
			if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || !(_resistance > 0.0f) || !(_speedConstant > 0.0f) || !(_torqueConstant > 0.0f)) return 0.0f;
			const float r = 0.5f * _wheelDiameter;
			const float stallForce = (_torqueConstant * (_voltage / _resistance) * _gearRatio) / r;
			if (F > stallForce) return 0.0f;
			const float vNoLoad = (r * _speedConstant * _voltage) / _gearRatio;
			const float v = (r * _speedConstant / _gearRatio) * (_voltage - (F * r * _resistance) / (_torqueConstant * _gearRatio));
			if (v < 0.0f) return 0.0f;
			if (v > vNoLoad) return vNoLoad;
			return v;
		}

		inline float getSpeedAtForceLimit(float maxTractiveForce)
		{
			return const_cast<const MotorEncoderDrive*>(this)->getSpeedAtForceLimit(maxTractiveForce);
		}

		/*inline float getSpeedAtForceLimit(float maxTractiveForce) const
		{
			const float F = (maxTractiveForce < 0.0f) ? -maxTractiveForce : maxTractiveForce;

			if (!(_wheelDiameter > 0.0f) || !(_gearRatio > 0.0f) || !(_resistance > 0.0f) || !(_speedConstant > 0.0f) || !(_torqueConstant > 0.0f))
			{
				return 0.0f;
			}

			const float r = 0.5f * _wheelDiameter;
			const float stallForce = (_torqueConstant * (_voltage / _resistance) * _gearRatio) / r;

			if (F > stallForce)
			{
				return 0.0f;
			}

			const float vNoLoad = (r * _speedConstant * _voltage) / _gearRatio;
			const float v = (r * _speedConstant / _gearRatio) * (_voltage - (F * r * _resistance) / (_torqueConstant * _gearRatio));

			if (v < 0.0f)
			{
				return 0.0f;
			}

			if (v > vNoLoad)
			{
				return vNoLoad;
			}

			return v;
		}*/
	};
}
