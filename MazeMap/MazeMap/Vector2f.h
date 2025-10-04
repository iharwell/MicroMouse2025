#pragma once
#include "defines.h"

#include <stdint.h>
namespace MazeMap
{
	class EXPORT Vector2f
	{
	private:
		float _x, _y;
	public:
		Vector2f();
		Vector2f(float x, float y);

		Vector2f& SetXY(float x, float y);

		float GetX();
		float GetX() const;
		float SetX(float x);

		float GetY();
		float GetY() const;
		float SetY(float y);

		float GetMagnitudeSquared();
		float GetMagnitudeSquared() const;

		float GetMagnitude();
		float GetMagnitude() const;
		float SetMagnitude(float magnitude);

		float GetAngle();
		float GetAngle() const;
		float SetAngle(float angle);

		float AngleTo(const Vector2f& target);
		float AngleTo(const Vector2f& target) const;

		Vector2f& RotateBy(const Vector2f& rotationVector);

		Vector2f& operator=(const Vector2f& other) noexcept;
		Vector2f& operator+=(const Vector2f& other) noexcept;
		Vector2f& operator-=(const Vector2f& other) noexcept;
		Vector2f& operator*=(float scalar) noexcept;
		Vector2f& operator/=(float scalar);

		Vector2f operator-() noexcept;
		Vector2f operator-() const noexcept;

		Vector2f operator+(const Vector2f& other) const noexcept;
		Vector2f operator-(const Vector2f& other) const noexcept;
		Vector2f operator*(float scalar) const noexcept;
		Vector2f operator/(float scalar) const;

		Vector2f operator+(const Vector2f& other) noexcept;
		Vector2f operator-(const Vector2f& other) noexcept;
		Vector2f operator*(float scalar) noexcept;
		Vector2f operator/(float scalar);

		float operator*(const Vector2f& other);
		float operator*(const Vector2f& other) const;
	};
}

MazeMap::Vector2f operator*(float scalar, const MazeMap::Vector2f& vec);