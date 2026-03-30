
#ifndef VECTORF_H
#define VECTORF_H
#include "defines.h"

#include <stdint.h>
#include <cmath>

namespace MazeMap
{
	template<int DIMENSION>
	class Vectorf
	{
	private:
		float _data[DIMENSION];
	public:
		Vectorf()
			: _data()
		{
		}

		Vectorf(const Vectorf<DIMENSION>&) = default;
		Vectorf(Vectorf<DIMENSION>&&) = default;

		float& operator[](int index) { return _data[index]; }
		const float& operator[](int index) const { return _data[index]; }

		float GetMagnitudeSquared() { return const_cast<const Vectorf<DIMENSION>*>(this)->GetMagnitudeSquared(); }
		float GetMagnitudeSquared() const
		{
			float runningSum = 0.0f;
			for (size_t i = 0; i < DIMENSION; i++)
			{
				runningSum += _data[i] * _data[i];
			}
			return runningSum;
		}

		float GetMagnitude() { return MazeMap::Math::Sqrtf(GetMagnitudeSquared()); }
		float GetMagnitude() const { return MazeMap::Math::Sqrtf(GetMagnitudeSquared()); }
		float SetMagnitude(float magnitude)
		{
			const float currentMagnitude = GetMagnitude();
			if (currentMagnitude <= 0.0f)
			{
				return 0.0f;
			}
			float k = magnitude / currentMagnitude;
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] *= k;
			}
			return GetMagnitude();
		}

		Vectorf<DIMENSION>& operator=(const Vectorf<DIMENSION>& other) noexcept
		{
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] = other._data[i];
			}
			return (*this);
		}
		Vectorf<DIMENSION>& operator+=(const Vectorf<DIMENSION>& other) noexcept
		{
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] += other._data[i];
			}
			return (*this);
		}
		Vectorf<DIMENSION>& operator-=(const Vectorf<DIMENSION>& other) noexcept
		{
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] -= other._data[i];
			}
			return (*this);
		}
		Vectorf<DIMENSION>& operator*=(float scalar) noexcept
		{
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] *= scalar;
			}
			return (*this);
		}
		Vectorf<DIMENSION>& operator/=(float scalar)
		{
			for (size_t i = 0; i < DIMENSION; i++)
			{
				_data[i] /= scalar;
			}
			return (*this);
		}

		Vectorf<DIMENSION> operator-() noexcept
		{
			return -(*const_cast<const Vectorf<DIMENSION>*>(this));
		}
		Vectorf<DIMENSION> operator-() const noexcept
		{
			Vectorf<DIMENSION> result(*this);
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result._data[i] = -result._data[i];
			}
			return result;
		}

		Vectorf<DIMENSION> operator+(const Vectorf<DIMENSION>& other) const noexcept
		{
			Vectorf<DIMENSION> result(*this);
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result._data[i] += other._data[i];
			}
			return result;
		}
		Vectorf<DIMENSION> operator-(const Vectorf<DIMENSION>& other) const noexcept
		{
			Vectorf<DIMENSION> result(*this);
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result._data[i] -= other._data[i];
			}
			return result;
		}
		Vectorf<DIMENSION> operator*(float scalar) const noexcept
		{
			Vectorf<DIMENSION> result(*this);
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result._data[i] *= scalar;
			}
			return result;
		}
		Vectorf<DIMENSION> operator/(float scalar) const
		{
			Vectorf<DIMENSION> result(*this);
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result._data[i] /= scalar;
			}
			return result;
		}

		Vectorf<DIMENSION> operator+(const Vectorf<DIMENSION>& other) noexcept
		{
			return (*const_cast<const Vectorf<DIMENSION>*>(this)) + other;
		}
		Vectorf<DIMENSION> operator-(const Vectorf<DIMENSION>& other) noexcept
		{
			return (*const_cast<const Vectorf<DIMENSION>*>(this)) - other;
		}
		Vectorf<DIMENSION> operator*(float scalar) noexcept
		{
			return (*const_cast<const Vectorf<DIMENSION>*>(this)) * scalar;
		}
		Vectorf<DIMENSION> operator/(float scalar)
		{
			return (*const_cast<const Vectorf<DIMENSION>*>(this)) / scalar;
		}

		float operator*(const Vectorf<DIMENSION>& other)
		{
			return (*const_cast<const Vectorf<DIMENSION>*>(this))* other;
		}
		float operator*(const Vectorf<DIMENSION>& other) const
		{
			float result = 0.0f;
			for (size_t i = 0; i < DIMENSION; i++)
			{
				result += _data[i] * other._data[i];
			}
			return result;
		}
	};

	template<>
	class Vectorf<2>
	{
	private:
		float _x, _y;
	public:
		Vectorf<2>()
			: _x(0.f)
			, _y(0.f)
		{
		}
		Vectorf<2>(const Vectorf<2>&) = default;
		Vectorf<2>(Vectorf<2>&&) = default;
		Vectorf<2>(float x, float y)
			: _x(x)
			, _y(y)
		{
		}

		float& operator[](int index)
		{
			if (index == 0)
			{
				return _x;
			}
			return _y;
		}
		const float& operator[](int index) const
		{
			if (index == 0)
			{
				return _x;
			}
			return _y;
		}

		Vectorf<2>& SetXY(float x, float y)
		{
			_x = x;
			_y = y;
			return (*this);
		}

		float GetX() { return const_cast<const Vectorf<2>*>(this)->GetX(); }
		float GetX() const
		{
			return _x;
		}
		float SetX(float x)
		{
			return (_x = x);
		}

		float GetY() { return const_cast<const Vectorf<2>*>(this)->GetY(); }
		float GetY() const
		{
			return _y;
		}
		float SetY(float y)
		{
			return (_y = y);
		}

		float GetMagnitudeSquared() { return const_cast<const Vectorf<2>*>(this)->GetMagnitudeSquared(); }
		float GetMagnitudeSquared() const
		{
			return _x * _x + _y * _y;
		}

		float GetMagnitude() { return const_cast<const Vectorf<2>*>(this)->GetMagnitude(); }
		float GetMagnitude() const
		{
			return MazeMap::Math::Sqrtf(GetMagnitudeSquared());
		}
		float SetMagnitude(float magnitude)
		{
			float scale = magnitude / GetMagnitude();
			SetX(_x * scale);
			SetY(_y * scale);
			return const_cast<const Vectorf<2>*>(this)->GetMagnitude();
		}

		float GetAngle() { return const_cast<const Vectorf<2>*>(this)->GetAngle(); }
		float GetAngle() const
		{
			return atan2f(_y, _x);
		}
		float SetAngle(float angle)
		{
			float mag = GetMagnitude();
			SetX(mag * cosf(angle));
			SetY(mag * sinf(angle));
			return const_cast<const Vectorf<2>*>(this)->GetAngle();
		}

		float AngleTo(const Vectorf<2>& target) { return const_cast<const Vectorf<2>*>(this)->AngleTo(target); }
		float AngleTo(const Vectorf<2>& target) const
		{
			// I'm treating this as a complex division to limit the trig calls we use.
			float x = (*this) * target;
			float y = this->_x * target._y - this->_y * target._x;
			return atan2f(y, x);
		}

		Vectorf<2>& RotateBy(const Vectorf<2>& rotationVector)
		{
			float mag = rotationVector.GetMagnitude();
			const float x = GetX();
			const float y = GetY();
			SetX((x * rotationVector.GetX() - y * rotationVector.GetY()) / mag);
			SetY((x * rotationVector.GetY() + y * rotationVector.GetX()) / mag);
			return (*this);
		}

		Vectorf<2>& operator=(const Vectorf<2>& other) noexcept
		{
			SetXY(other.GetX(), other.GetY());
			return (*this);
		}
		Vectorf<2>& operator+=(const Vectorf<2>& other) noexcept
		{
			SetXY(GetX() + other.GetX(), GetY() + other.GetY());
			return (*this);
		}
		Vectorf<2>& operator-=(const Vectorf<2>& other) noexcept
		{
			SetXY(GetX() - other.GetX(), GetY() - other.GetY());
			return (*this);
		}
		Vectorf<2>& operator*=(float scalar) noexcept
		{
			SetXY(GetX() * scalar, GetY() * scalar);
			return (*this);
		}
		Vectorf<2>& operator/=(float scalar)
		{
			SetXY(GetX() / scalar, GetY() / scalar);
			return (*this);
		}

		Vectorf<2> operator-() noexcept { return -(*const_cast<const Vectorf<2>*>(this)); }
		Vectorf<2> operator-() const noexcept
		{
			return Vectorf<2>(-GetX(), -GetY());
		}

		Vectorf<2> operator+(const Vectorf<2>& other) const noexcept
		{
			return Vectorf<2>(this->_x + other._x, this->_y + other._y);
		}
		Vectorf<2> operator-(const Vectorf<2>& other) const noexcept
		{
			return Vectorf<2>(this->_x - other._x, this->_y - other._y);
		}
		Vectorf<2> operator*(float scalar) const noexcept
		{
			return Vectorf<2>(_x * scalar, _y * scalar);
		}
		Vectorf<2> operator/(float scalar) const
		{
			return Vectorf<2>(_x / scalar, _y / scalar);
		}

		Vectorf<2> operator+(const Vectorf<2>& other) noexcept { return (*const_cast<const Vectorf<2>*>(this)) + other; }
		Vectorf<2> operator-(const Vectorf<2>& other) noexcept { return (*const_cast<const Vectorf<2>*>(this)) - other; }
		Vectorf<2> operator*(float scalar) noexcept { return (*const_cast<const Vectorf<2>*>(this)) * scalar; }
		Vectorf<2> operator/(float scalar) { return (*const_cast<const Vectorf<2>*>(this)) / scalar; }

		float operator*(const Vectorf<2>& other) { return (*const_cast<const Vectorf<2>*>(this)) * other; }
		float operator*(const Vectorf<2>& other) const
		{
			return _x * other._x + _y * other._y;
		}
	};
}

inline MazeMap::Vectorf<2> operator*(float scalar, const MazeMap::Vectorf<2>& vec)
{
	return vec * scalar;
}
#endif

