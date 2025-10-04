#include "pch.h"
#include "Vector2f.h"
#include "math.h"

namespace MazeMap
{
	MazeMap::Vector2f::Vector2f()
		: _x(0.f)
		, _y(0.f)
	{}

	MazeMap::Vector2f::Vector2f(float x, float y)
		: _x(x)
		, _y(y)
	{
	}

	Vector2f& MazeMap::Vector2f::SetXY(float x, float y)
	{
		_x = x;
		_y = y;
		return (*this);
	}

	float Vector2f::GetX() { return const_cast<const Vector2f*>(this)->GetX(); }

	float MazeMap::Vector2f::GetX() const
	{
		return _x;
	}

	float MazeMap::Vector2f::SetX(float x)
	{
		return (_x=x);
	}

	float Vector2f::GetY() { return const_cast<const Vector2f*>(this)->GetY(); }

	float MazeMap::Vector2f::GetY() const
	{
		return _y;
	}

	float Vector2f::SetY(float y)
	{
		return (_y = y);
	}

	float Vector2f::GetMagnitudeSquared() { return const_cast<const Vector2f*>(this)->GetMagnitudeSquared(); }

	float Vector2f::GetMagnitudeSquared() const
	{
		return _x * _x + _y * _y;
	}

	float Vector2f::GetMagnitude() { return const_cast<const Vector2f*>(this)->GetMagnitude(); }

	float Vector2f::GetMagnitude() const
	{
		return sqrtf(GetMagnitudeSquared());
	}
	float Vector2f::SetMagnitude(float magnitude)
	{
		float scale = magnitude/GetMagnitude();
		SetX(_x * scale);
		SetY(_y * scale);
		return const_cast<const Vector2f*>(this)->GetMagnitude();
	}

	float Vector2f::GetAngle() { return const_cast<const Vector2f*>(this)->GetAngle(); }

	float Vector2f::GetAngle() const
	{
		return atan2f(_y, _x);
	}
	float Vector2f::SetAngle(float angle)
	{
		float mag = GetMagnitude();
		SetX(mag * cosf(angle));
		SetY(mag * sinf(angle));
		return const_cast<const Vector2f*>(this)->GetAngle();
	}

	float Vector2f::AngleTo(const Vector2f& target) { return const_cast<const Vector2f*>(this)->AngleTo(target); }

	float Vector2f::AngleTo(const Vector2f& target) const
	{
		// I'm treating this as a complex division to limit the trig calls we use.
		float x = (*this)*target;
		float y = this->_x * target._y - this->_y * target._x;
		return atan2f(y, x);
	}

	Vector2f& Vector2f::RotateBy(const Vector2f& rotationVector)
	{
		float mag = rotationVector.GetMagnitude();
		SetX((GetX() * rotationVector.GetX() - GetY() * rotationVector.GetY()) / mag);
		SetY((GetX() * rotationVector.GetY() + GetY() * rotationVector.GetX()) / mag);
		return (*this);
	}

	Vector2f& Vector2f::operator=(const Vector2f& other) noexcept
	{
		SetXY(other.GetX(), other.GetY());
		return (*this);
	}
	Vector2f& Vector2f::operator+=(const Vector2f& other) noexcept
	{
		SetXY(GetX()+other.GetX(), GetY()+other.GetY());
		return (*this);
	}
	Vector2f& Vector2f::operator-=(const Vector2f& other) noexcept
	{
		SetXY(GetX() - other.GetX(), GetY() - other.GetY());
		return (*this);
	}
	Vector2f& Vector2f::operator*=(float scalar) noexcept
	{
		SetXY(GetX() * scalar, GetY() * scalar);
		return (*this);
	}
	Vector2f& Vector2f::operator/=(float scalar)
	{
		SetXY(GetX() / scalar, GetY() / scalar);
		return (*this);
	}

	Vector2f Vector2f::operator-() const noexcept
	{
		return Vector2f(-GetX(),-GetY());
	}

	Vector2f Vector2f::operator+(const Vector2f& other) const noexcept
	{
		return Vector2f(this->_x + other._x, this->_y + other._y);
	}


	Vector2f Vector2f::operator-(const Vector2f& other) const noexcept
	{
		return Vector2f(this->_x - other._x, this->_y - other._y);
	}



	Vector2f Vector2f::operator*(float scalar) const noexcept
	{
		return Vector2f(_x * scalar, _y * scalar);
	}

	Vector2f Vector2f::operator/(float scalar) const
	{
		return Vector2f(_x / scalar, _y / scalar);
	}



	float Vector2f::operator*(const Vector2f& other) const
	{
		return _x * other._x + _y * other._y;
	}

	Vector2f Vector2f::operator-() noexcept                      { return -(*const_cast<const Vector2f*>(this)); }
	Vector2f Vector2f::operator+(const Vector2f& other) noexcept { return (*const_cast<const Vector2f*>(this)) + other; }
	Vector2f Vector2f::operator-(const Vector2f& other) noexcept { return (*const_cast<const Vector2f*>(this)) - other; }
	Vector2f Vector2f::operator*(float scalar) noexcept          { return (*const_cast<const Vector2f*>(this)) * scalar; }
	Vector2f Vector2f::operator/(float scalar)                   { return (*const_cast<const Vector2f*>(this)) / scalar; }
	float Vector2f::operator*(const Vector2f& other)             { return (*const_cast<const Vector2f*>(this)) * other; }

}

MazeMap::Vector2f operator*(float scalar, const MazeMap::Vector2f& vec)
{
	return vec * scalar;
}
