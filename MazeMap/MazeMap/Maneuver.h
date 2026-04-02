#ifndef MANEUVER_H
#define MANEUVER_H
// Declares the maneuver catalogue and shared turn-profile helpers used by path execution and motion planning.
#include "Defines.h"
#include "MazeLocation.h"
#include "DirectionalLocation.h"
#include "Maze.h"
#include "Vehicle.h"
#include "Kinematics.h"

namespace MazeMap
{
	MAZEMAP_INLINE bool IsFiniteFloat(const float value) noexcept
	{
#if defined(ARDUINO) || defined(CORE_TEENSY) || defined(ARDUINO_TEENSY41)
		return isfinite(value);
#else
		return std::isfinite(value);
#endif
	}

	MAZEMAP_INLINE float AbsFloat(const float value) noexcept
	{
#if defined(ARDUINO) || defined(CORE_TEENSY) || defined(ARDUINO_TEENSY41)
		return fabs(value);
#else
		return std::fabs(value);
#endif
	}

	MAZEMAP_INLINE float ClampFloat(const float value, const float low, const float high) noexcept
	{
		return (value < low) ? low : ((value > high) ? high : value);
	}

	struct SmoothTurnExecutionProfile
	{
		float radius = 0.0f;
		float radians = 0.0f;
		float turnInDistance = 0.0f;
		float preTurnDistance = 0.0f;
		float constantTurnDistance = 0.0f;
		float postTurnDistance = 0.0f;
		float totalDistance = 0.0f;

		MAZEMAP_INLINE bool IsValid() const noexcept
		{
			return IsFiniteFloat(radius) &&
				(radius > 0.0f) &&
				IsFiniteFloat(radians) &&
				(AbsFloat(radians) > 0.0f) &&
				IsFiniteFloat(turnInDistance) &&
				(turnInDistance >= 0.0f) &&
				IsFiniteFloat(preTurnDistance) &&
				(preTurnDistance >= 0.0f) &&
				IsFiniteFloat(constantTurnDistance) &&
				(constantTurnDistance >= 0.0f) &&
				IsFiniteFloat(postTurnDistance) &&
				(postTurnDistance >= 0.0f) &&
				IsFiniteFloat(totalDistance) &&
				(totalDistance > 0.0f);
		}
	};

	MAZEMAP_INLINE SmoothTurnExecutionProfile ScaleSmoothTurnExecutionProfile(
		const SmoothTurnExecutionProfile& profile,
		float distanceScale) noexcept
	{
		if (!(IsFiniteFloat(distanceScale) && (distanceScale > 0.0f)))
		{
			return SmoothTurnExecutionProfile{};
		}

		SmoothTurnExecutionProfile scaled = profile;
		scaled.radius *= distanceScale;
		scaled.turnInDistance *= distanceScale;
		scaled.preTurnDistance *= distanceScale;
		scaled.constantTurnDistance *= distanceScale;
		scaled.postTurnDistance *= distanceScale;
		scaled.totalDistance *= distanceScale;
		return scaled;
	}

	MAZEMAP_INLINE bool TryComputeSmoothTurnTarget(
		const SmoothTurnExecutionProfile& profile,
		float traveledDistance,
		float forwardSpeed,
		float& yawOffsetRad,
		float& angularVelocityRadps) noexcept
	{
		yawOffsetRad = 0.0f;
		angularVelocityRadps = 0.0f;
		if (!profile.IsValid() ||
			!IsFiniteFloat(traveledDistance) ||
			!IsFiniteFloat(forwardSpeed))
		{
			return false;
		}

		const float clampedDistance = ClampFloat(traveledDistance, 0.0f, profile.totalDistance);
		const float turnSign = (profile.radians < 0.0f) ? -1.0f : 1.0f;
		const float radius = profile.radius;
		const float rampDistance = profile.turnInDistance;
		const float preTurnEnd = profile.preTurnDistance;
		const float rampInEnd = preTurnEnd + rampDistance;
		const float constantEnd = rampInEnd + profile.constantTurnDistance;
		const float rampOutEnd = constantEnd + rampDistance;

		if (clampedDistance <= preTurnEnd)
		{
			return true;
		}

		if (rampDistance <= 0.0f)
		{
			if (clampedDistance <= constantEnd)
			{
				const float curveDistance = clampedDistance - preTurnEnd;
				yawOffsetRad = turnSign * (curveDistance / radius);
				angularVelocityRadps = turnSign * (forwardSpeed / radius);
			}
			else
			{
				yawOffsetRad = profile.radians;
			}
			return true;
		}

		if (clampedDistance <= rampInEnd)
		{
			const float rampDistanceTravelled = clampedDistance - preTurnEnd;
			yawOffsetRad = turnSign * ((rampDistanceTravelled * rampDistanceTravelled) / (2.0f * rampDistance * radius));
			angularVelocityRadps = turnSign * forwardSpeed * (rampDistanceTravelled / (rampDistance * radius));
			return true;
		}

		if (clampedDistance <= constantEnd)
		{
			const float curveDistanceTravelled = clampedDistance - rampInEnd;
			yawOffsetRad = turnSign * ((0.5f * rampDistance) + curveDistanceTravelled) / radius;
			angularVelocityRadps = turnSign * (forwardSpeed / radius);
			return true;
		}

		if (clampedDistance <= rampOutEnd)
		{
			const float rampDistanceTravelled = clampedDistance - constantEnd;
			yawOffsetRad = turnSign *
				((0.5f * rampDistance) +
					profile.constantTurnDistance +
					rampDistanceTravelled -
					((rampDistanceTravelled * rampDistanceTravelled) / (2.0f * rampDistance))) /
				radius;
			angularVelocityRadps = turnSign * forwardSpeed * (1.0f - (rampDistanceTravelled / rampDistance)) / radius;
			return true;
		}

		yawOffsetRad = profile.radians;
		return true;
	}

	enum ManeuverCode : uint8_t
	{
		// Before hand-building any path from the codes below, first validate it against the known walls with
		// ManeuverSet::GetSet().IsValidMove(code, start, maze), then confirm the evaluated endpoint with
		// ManeuverSet::GetSet().Move(code, start) or ManeuverInstance::GetEnd().
		// The code name alone does not prove the start/end geometry does what you intend.
		MC_NONE = 0,
		S1 = 1,
		S2 = 2,
		S3 = 3,
		S4 = 4,
		S5 = 5,
		S6 = 6,
		S7 = 7,
		S8 = 8,
		S9 = 9,
		S10 = 10,
		S11 = 11,
		S12 = 12,
		S13 = 13,
		S14 = 14,
		S15 = 15,
		S16 = 16,
		S17 = 17,
		S18 = 18,
		S19 = 19,
		S20 = 20,
		S21 = 21,
		S22 = 22,
		S23 = 23,
		S24 = 24,
		S25 = 25,
		S26 = 26,
		S27 = 27,
		S28 = 28,
		S29 = 29,
		S30 = 30,
		S31 = 31,

		IP45 = 32,
		IP90 = 33,
		IP135 = 34,
		IP180 = 35,
		S45SS = 36,
		S45SD = 37,
		S45LS = 38,
		S45LD = 39,
		S90SS = 40,
		S90SD = 41,
		S90LS = 42,
		S90LD = 43,
		S135SS = 44,
		S135SD = 45,
		S135LS = 46,
		S135LD = 47,
		S180SS = 48,
		S180LS = 49,
		S90ELD = 50,
		S180ELS = 51,

		MIRRORED_MANEUVER_FLAG = 0b10000000,
		INVERTED_MIRRORED_MANEUVER_FLAG = 0b01111111,

		IP45_M = IP45 | MIRRORED_MANEUVER_FLAG,
		IP90_M = IP90 | MIRRORED_MANEUVER_FLAG,
		IP135_M = IP135 | MIRRORED_MANEUVER_FLAG,
		IP180_M = IP180 | MIRRORED_MANEUVER_FLAG,
		S45SD_M = S45SD | MIRRORED_MANEUVER_FLAG,
		S45SS_M = S45SS | MIRRORED_MANEUVER_FLAG,
		S45LS_M = S45LS | MIRRORED_MANEUVER_FLAG,
		S45LD_M = S45LD | MIRRORED_MANEUVER_FLAG,
		S90SS_M = S90SS | MIRRORED_MANEUVER_FLAG,
		S90SD_M = S90SD | MIRRORED_MANEUVER_FLAG,
		S90LS_M = S90LS | MIRRORED_MANEUVER_FLAG,
		S90LD_M = S90LD | MIRRORED_MANEUVER_FLAG,
		S135SS_M = S135SS | MIRRORED_MANEUVER_FLAG,
		S135SD_M = S135SD | MIRRORED_MANEUVER_FLAG,
		S135LS_M = S135LS | MIRRORED_MANEUVER_FLAG,
		S135LD_M = S135LD | MIRRORED_MANEUVER_FLAG,
		S180SS_M = S180SS | MIRRORED_MANEUVER_FLAG,
		S180LS_M = S180LS | MIRRORED_MANEUVER_FLAG,
		S90ELD_M = S90ELD | MIRRORED_MANEUVER_FLAG,
		S180ELS_M = S180ELS | MIRRORED_MANEUVER_FLAG,
	};

	inline ManeuverCode operator|(ManeuverCode a, ManeuverCode b)
	{
		return static_cast<ManeuverCode>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
	}
	inline ManeuverCode operator&(ManeuverCode a, ManeuverCode b)
	{
		return static_cast<ManeuverCode>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
	}
	inline ManeuverCode operator^(ManeuverCode a, ManeuverCode b)
	{
		return static_cast<ManeuverCode>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b));
	}
	inline constexpr int16_t CodeDegrees(ManeuverCode mc)
	{
		if (mc < 32)
		{
			return 0;
		}

		// Unmirrored smooth-turn codes are defined as right-hand turns in the maneuver set.
		// Project yaw uses positive angles for clockwise/right rotation.
		const int16_t mult = ((mc & MIRRORED_MANEUVER_FLAG) == MIRRORED_MANEUVER_FLAG) ? static_cast<int16_t>(-1) : static_cast<int16_t>(1);

		const ManeuverCode normCode = mc & INVERTED_MIRRORED_MANEUVER_FLAG;

		switch (normCode)
		{
		case IP45:
		case S45SS:
		case S45SD:
		case S45LS:
		case S45LD:
			return 45 * mult;
		case IP90:
		case S90SS:
		case S90SD:
		case S90LS:
		case S90LD:
		case S90ELD:
			return 90 * mult;
		case IP135:
		case S135SS:
		case S135SD:
		case S135LS:
		case S135LD:
			return 135 * mult;
		case IP180:
		case S180SS:
		case S180LS:
		case S180ELS:
			return 180 * mult;
		default:
			return 0;
		}
	}

	class Maneuver
	{
		//↑↗→↘↓↙←↖
	public:
		virtual ~Maneuver() = default;
		virtual bool SupportsDiagonalEntry() const = 0; //
		virtual bool SupportsStraightEntry() const = 0; //

		virtual uint8_t GetStepCount() const = 0; //
		virtual RelativeDirectionalDistance GetStep(uint8_t index) const = 0; //

		virtual float GetCost(const Vehicle& vehicle) const = 0; //
		virtual float GetEntrySpeed(const Vehicle& vehicle) const = 0; //
		virtual float GetExitSpeed(const Vehicle& vehicle) const = 0; //
		virtual float GetNominalTurnRadiusInCells() const { return 0.0f; }
		virtual bool TryGetSmoothTurnExecutionProfile(SmoothTurnExecutionProfile& profile) const
		{
			profile = SmoothTurnExecutionProfile{};
			return false;
		}

		virtual ManeuverCode GetManeuverID() const = 0;
		virtual ManeuverCode GetBackwardsManeuverID() const = 0;

		virtual float GetVMax(const Vehicle& vehicle) const = 0; //
		//virtual float GetTurnRadius(const Vehicle& vehicle) const = 0;
		//virtual float GetTurnInLength(const Vehicle& vehicle) const = 0;


		MAZEMAP_INLINE bool IsValidMove(DirectionalLocation start, const Maze& maze, bool mirrored) const
		{
			if (IsDiagonal(start.GetDirection()) && !SupportsDiagonalEntry())
			{
				return false;
			}
			if (!IsDiagonal(start.GetDirection()) && !SupportsStraightEntry())
			{
				return false;
			}
			//if (!maze.IsAccessibleLocation(start.GetLocation()))
			//{
			//	return false;
			//}
			DirectionalLocation current = start;
			uint8_t stepCount = GetStepCount();
			for (uint8_t i = 0; i < stepCount; i++)
			{
				RelativeDirectionalDistance rdd = GetStep(i);
				RelativeDirection direction = rdd.GetDirection();
				if (mirrored)
				{
					direction = -direction;
				}
				DirectionalLocation target = current >> rdd;
				current = current >> direction;
				uint8_t dist = rdd.GetDistance();
				for (uint8_t j = 0; j < dist; j++)
				{
					current = current.MoveForward(1);
					if (!maze.IsAccessibleLocation(current.GetLocation()))
					{
						return false;
					}
					if (current == target)
					{
						break;
					}
				}
			}
			return true;
		}

		MAZEMAP_INLINE DirectionalLocation Move(DirectionalLocation start, bool mirrored) const
		{
			DirectionalLocation result = start;
			for (uint8_t i = 0; i < GetStepCount(); i++)
			{
				RelativeDirectionalDistance rdd = GetStep(i);
				if (mirrored)
				{
					rdd.SetDirection(-rdd.GetDirection());
				}
				result = result.Turn(rdd.GetDirection());
				result = result.MoveForward(rdd.GetDistance());
			}
			return result;
		}

		MAZEMAP_INLINE uint8_t DistanceTravelled() const
		{
			uint8_t dist = 0;
			
			for (uint8_t i = 0; i < GetStepCount(); i++)
			{
				dist += GetStep(i).GetDistance();
			}
			return dist;
		}
	};

	template <int STEPSIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class SimpleManeuver : public Maneuver
	{
	protected:
		RelativeDirectionalDistance _instructions[STEPSIZE];
	public:
		virtual MAZEMAP_INLINE bool SupportsDiagonalEntry() const override { return DIAGONAL_ENTRY; };
		virtual MAZEMAP_INLINE bool SupportsStraightEntry() const override { return STRAIGHT_ENTRY; };

		virtual MAZEMAP_INLINE uint8_t GetStepCount() const override { return STEPSIZE; }
		virtual MAZEMAP_INLINE RelativeDirectionalDistance GetStep(uint8_t index) const override { return _instructions[index]; }
	};

	template <int STEPSIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class SmoothTurnManeuver : public SimpleManeuver<STEPSIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
	{
	private:
		// The turn radius in cells
		float _radius_in_cells;
		// The angle turned
		float _radians;
		// The distance in cells travelled during the entry/exit cycloids of the turn.
		float _turnInDelta;
		// The distance travelled after exiting the turn before reaching the next maneuver.
		float _preTurnDist;
		// The distance travelled after reaching the start of the maneuver, but before altering direction.
		float _postTurnDist;
	protected:
		MAZEMAP_INLINE SmoothTurnManeuver(float radius_in_cells, float radians, float turnInDelta, float preTurnDist, float postTurnDist)
			: _radius_in_cells(radius_in_cells)
			, _radians(radians)
			, _turnInDelta(turnInDelta)
			, _preTurnDist(preTurnDist)
			, _postTurnDist(postTurnDist)
		{
		}

		MAZEMAP_INLINE float GetArcLengthInCells() const
		{
			//float angleChangeInDeltas = _turnInDelta / _radius_in_cells;
			//float arcRadians = _radians - angleChangeInDeltas;
			//float arcDist = arcRadians * _radius_in_cells;

			return _radians * _radius_in_cells - _turnInDelta;
		}
	public:
		MAZEMAP_INLINE float GetRadians() const { return _radians; }
		MAZEMAP_INLINE float GetRadiusInCells() const { return _radius_in_cells; }
		MAZEMAP_INLINE float GetTurnInDistInCells() const { return _turnInDelta; }
		MAZEMAP_INLINE float GetPreTurnDistInCells() const { return _preTurnDist; }
		MAZEMAP_INLINE float GetPostTurnDistInCells() const { return _postTurnDist; }

		MAZEMAP_INLINE float GetTravelDistInCells() const
		{
			//return _preTurnDist + 2.0f * _turnInDelta + GetArcLengthInCells() + _postTurnDist;
			return _preTurnDist + _turnInDelta + _radians * _radius_in_cells + _postTurnDist;
		}
        virtual MAZEMAP_INLINE float GetVMax(const Vehicle& vehicle) const override
        {
            const float cellSize = Maze::GetCellDimension() / 100.0f;
            return vehicle.GetTurnSpeed(GetRadiusInCells() * cellSize);
        }
        virtual MAZEMAP_INLINE float GetCost(const Vehicle& vehicle) const override
        {
            const float cellSize = Maze::GetCellDimension() / 100.0f;
            float turnSpeed = vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
            float turnCost = cellSize * GetTravelDistInCells() / turnSpeed;
            return turnCost;
        }
		virtual MAZEMAP_INLINE float GetEntrySpeed(const Vehicle& vehicle) const override
		{
			//const float cellSize = Maze::GetCellDimension();
			//return vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
			return GetVMax(vehicle);
		}
		virtual MAZEMAP_INLINE float GetExitSpeed(const Vehicle& vehicle) const override
		{
			//const float cellSize = Maze::GetCellDimension();
			//return vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
			return GetVMax(vehicle);
		}
		virtual MAZEMAP_INLINE float GetNominalTurnRadiusInCells() const override { return _radius_in_cells; }
		virtual MAZEMAP_INLINE bool TryGetSmoothTurnExecutionProfile(SmoothTurnExecutionProfile& profile) const override
		{
			profile.radius = _radius_in_cells;
			profile.radians = _radians;
			profile.turnInDistance = _turnInDelta;
			profile.preTurnDistance = _preTurnDist;
			profile.constantTurnDistance = GetArcLengthInCells();
			profile.postTurnDistance = _postTurnDist;
			profile.totalDistance = GetTravelDistInCells();
			return profile.IsValid();
		}
	};
	/*
	template <int STEPSIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class SimpleSingleTurnManeuver : public SimpleManeuver<STEPSIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
	{
	private:
		const float _radius_in_cells;
		const float _radians;
	protected:
		SimpleSingleTurnManeuver(float radius_in_cells, float radians)
			: _radius_in_cells(radius_in_cells)
			, _radians(radians)
		{}
	public:
		MAZEMAP_INLINE float GetRadians() const { return _radians; }
		MAZEMAP_INLINE float GetRadiusInCells() const { return _radius_in_cells; }

		virtual float GetCost(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
			float turnCost = turnSpeed * cellSize * _radius_in_cells * _radians;
			return turnCost;
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const override
		{ return vehicle.GetTurnSpeed(cellSize * _radius_in_cells); }
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize) const override
		{ return vehicle.GetTurnSpeed(cellSize * _radius_in_cells); }
	};

	template <int SIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class ComplexSingleTurnManeuver : public SimpleSingleTurnManeuver<SIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
	{
	private:
		float _entryLength;
		float _exitLength;
	protected:
		ComplexSingleTurnManeuver(float radius_in_cells, float radians, float entryLength, float exitLength)
			: SimpleSingleTurnManeuver<SIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>(radius_in_cells, radians)
			, _entryLength(entryLength)
			, _exitLength(exitLength)
		{
		}
	public:

		virtual float GetCost(const Vehicle& vehicle, float cellSize) const override
		{
			float vt = vehicle.GetTurnSpeed(cellSize * this->GetRadiusInCells());
			float a = vehicle.GetMaxForwardAcceleration();
			//float entryCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _entryLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			//float exitCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _exitLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			float entryCost = LinearKinematics::TIgnoringV1(_entryLength * cellSize, vt, a);
			float exitCost = LinearKinematics::TIgnoringV1(_exitLength * cellSize, vt, a);
			return RotationalKinematics::TGivenCentVtTheta(vehicle.GetMaxLateralAcceleration(), vt, this->GetRadians()) + entryCost + exitCost;
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * this->GetRadiusInCells());
			float a = vehicle.GetMaxForwardAcceleration();
			return LinearKinematics::V1IgnoringT(_entryLength * cellSize, turnSpeed, a);

		}
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * this->GetRadiusInCells());
			float a = vehicle.GetMaxForwardAcceleration();
			return sqrtf(turnSpeed * turnSpeed + 2.0f * a * cellSize * _exitLength);
		}
	};
	*/
	//↗0
	class TurnInPlace : public SimpleManeuver<1, true, true>
	{
		const float _turnRatio;
	public:
		MAZEMAP_INLINE TurnInPlace( float pisTurned, RelativeDirectionalDistance instruction)
			: _turnRatio(pisTurned)
		{
			_instructions[0] = instruction;
		}

		virtual MAZEMAP_INLINE float GetVMax(const Vehicle& vehicle) const override
		{
			(void)vehicle;
			return 0.0f;
		};
		/*virtual float GetTurnRadius(const Vehicle& vehicle) const override
		{
			return 0.0f;
		};
		virtual float GetTurnInLength(const Vehicle& vehicle) const override
		{
			return 0.0f;
		};*/

		virtual MAZEMAP_INLINE float GetCost(const Vehicle& vehicle)  const override
		{
			return vehicle.GetInPlaceTurnTime(PI_F * _turnRatio);
		}
		virtual MAZEMAP_INLINE float GetEntrySpeed(const Vehicle& vehicle) const override
		{
			(void)vehicle;
			return 0.0f;
		}
		virtual MAZEMAP_INLINE float GetExitSpeed(const Vehicle& vehicle)  const override
		{
			(void)vehicle;
			return 0.0f;
		}
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return GetManeuverID() | MIRRORED_MANEUVER_FLAG; }
	};

	//↗0
	class TurnInPlace45 : public TurnInPlace
	{
	public:
		MAZEMAP_INLINE TurnInPlace45()
			: TurnInPlace(0.25f, RelativeDirectionalDistance(Right45, 0))
		{}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return IP45; }
	};
	//→0
	class TurnInPlace90 : public TurnInPlace
	{
	public:
		MAZEMAP_INLINE TurnInPlace90()
			: TurnInPlace(0.5f, RelativeDirectionalDistance(Right90, 0))
		{
		}

		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return IP90; }
	};
	//↘0
	class TurnInPlace135 : public TurnInPlace
	{
	public:
		MAZEMAP_INLINE TurnInPlace135()
			: TurnInPlace(0.75f, RelativeDirectionalDistance(Right135, 0))
		{}

		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return IP135; }
	};
	//↓0
	class TurnInPlace180 : public TurnInPlace
	{
	public:
		MAZEMAP_INLINE TurnInPlace180()
			: TurnInPlace(1.0f, RelativeDirectionalDistance(Reverse, 0))
		{
		}

		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return IP180; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return IP180; }
	};


	//↑1 ↗1
	class Smooth45ShortStraight : public SmoothTurnManeuver<2,true,false>
	{
	public:
		MAZEMAP_INLINE Smooth45ShortStraight()
			: SmoothTurnManeuver(156/180.0f, PI_F / 4.0f, 32/180.0f, 9/180.0f,42.3f/180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S45SS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S45SD | MIRRORED_MANEUVER_FLAG; }
	};

	//↑1 ↗1
	class Smooth45ShortDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		MAZEMAP_INLINE Smooth45ShortDiagonal()
			: SmoothTurnManeuver(
				156 / 180.0f,
				PI_F / 4.0f,
				32 / 180.0f,
				42.3f / 180.0f,
				9 / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S45SD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S45SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 ↗1
	class Smooth45LongStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:

		MAZEMAP_INLINE Smooth45LongStraight()
			: SmoothTurnManeuver(
				246 / 180.0f,
				PI_F / 4.0f,
				30 / 180.0f,
				64.0f / 180.0f,
				10.3f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 2);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S45LS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S45LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗2
	class Smooth45LongDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		MAZEMAP_INLINE Smooth45LongDiagonal()
			: SmoothTurnManeuver(
				246 / 180.0f,
				PI_F / 4.0f,
				30 / 180.0f,
				10.3f / 180.0f,
				64.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 2);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S45LD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S45LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 ↗0
	class Smooth90ShortStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:
		MAZEMAP_INLINE Smooth90ShortStraight()
			: SmoothTurnManeuver(
				63 / 180.0f,
				PI_F / 2.0f,
				33 / 180.0f,
				10.0f / 180.0f,
				10.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 0);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90SS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1
	class Smooth90ShortDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		MAZEMAP_INLINE Smooth90ShortDiagonal()
			: SmoothTurnManeuver(
				79 / 180.0f,
				PI_F / 2.0f,
				37 / 180.0f,
				10.27f / 180.0f,
				10.27f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90SD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 →2
	/*class Smooth90LongDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		Smooth90LongDiagonal()
			: SmoothTurnManeuver(
				79 / 180.0f,
				PI_F / 2.0f,
				37 / 180.0f,
				10.27f / 180.0f,
				10.27f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 2);
			_instructions[1] = RelativeDirectionalDistance(Right90, 2);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90LD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90LD | MIRRORED_MANEUVER_FLAG; }
	};*/
	//↑1 ↗1 ↗1
	class Smooth90LongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		MAZEMAP_INLINE Smooth90LongStraight()
			: SmoothTurnManeuver(
				153 / 180.0f,
				PI_F / 2.0f,
				36 / 180.0f,
				9.0f / 180.0f,
				9.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90LS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90LS | MIRRORED_MANEUVER_FLAG; }
	};


	/*//↑5 ↗1 ↗5
	class Smooth90ExtraLongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		Smooth90ExtraLongStraight()
			: SmoothTurnManeuver(1.503f, 1.79245f, 1.517f, 1.517f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 5);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90ELS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90ELS | MIRRORED_MANEUVER_FLAG; }
	};*/
	//↑1 ↗2 ↗1
	class Smooth90ExtraLongDiagonal : public SmoothTurnManeuver<3, true, false>
	{
	public:
		MAZEMAP_INLINE Smooth90ExtraLongDiagonal()
			: SmoothTurnManeuver(
				225 / 180.0f,
				PI_F / 2.0f,
				38 / 180.0f,
				10.558f / 180.0f,
				10.558f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 5);
		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S90ELD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S90ELD | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 →1
	class Smooth135ShortStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		MAZEMAP_INLINE Smooth135ShortStraight()
			: SmoothTurnManeuver(
				63 / 180.0f,
				PI_F * 3.0f / 4.0f,
				33 / 180.0f,
				10.0f / 180.0f,
				84.279f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);

		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S135SS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S135SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗0
	class Smooth135ShortDiagonal : public SmoothTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		MAZEMAP_INLINE Smooth135ShortDiagonal()
			: SmoothTurnManeuver(
				63 / 180.0f,
				PI_F * 3.0f / 4.0f,
				33 / 180.0f,
				84.279f / 180.0f,
				10.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 0);

		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S135SD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S135SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1
	class Smooth135LongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		MAZEMAP_INLINE Smooth135LongStraight()
			: SmoothTurnManeuver(
				86 / 180.0f,
				PI_F * 3.0f / 4.0f,
				60 / 180.0f,
				28.0f / 180.0f,
				13.28f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);

		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S135LS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S135LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗1
	class Smooth135LongDiagonal : public SmoothTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		MAZEMAP_INLINE Smooth135LongDiagonal()
			: SmoothTurnManeuver(
				86 / 180.0f,
				PI_F* 3.0f / 4.0f,
				60 / 180.0f,
				13.28f / 180.0f,
				28.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 1);

		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S135LD; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S135LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 →1 ↗0
	class Smooth180ShortStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		MAZEMAP_INLINE Smooth180ShortStraight()
			: SmoothTurnManeuver(
				89 / 180.0f,
				PI_F,
				66 / 180.0f,
				10.0f / 180.0f,
				10.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 0);

		}
		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S180SS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S180SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1 ↗1
	class Smooth180LongStraight : public SmoothTurnManeuver<4, true, false>
	{
	private:
		float GetLeadingDistance(const Vehicle& vehicle, float cellSize) const
		{
			return (cellSize - vehicle.GetWidth()- WALL_THICKNESS - MIN_CLEARANCE) * 0.5f;
		}
		float GetEntryDistance(const Vehicle& vehicle, float cellSize) const
		{
			return (0.5f * cellSize) - (cellSize - vehicle.GetWidth() - WALL_THICKNESS - MIN_CLEARANCE) * 0.5f;
		}
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		MAZEMAP_INLINE Smooth180LongStraight()
			: SmoothTurnManeuver(
				88 / 180.0f,
				PI_F,
				66 / 180.0f,
				60.0f / 180.0f,
				60.0f / 180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);
			_instructions[3] = RelativeDirectionalDistance(Right45, 1);

		}

		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S180LS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S180LS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑5 ↗1 →1 ↗5
	class Smooth180ExtraLongStraight : public SimpleManeuver<4, true, false>
	{
	private:
		float GetLeadingDistance(const Vehicle& vehicle, float cellSize) const
		{
			return (cellSize - vehicle.GetWidth() - WALL_THICKNESS - MIN_CLEARANCE) * 0.75f;
		}
		float GetEntryDistance(const Vehicle& vehicle, float cellSize) const
		{
			(void)vehicle;
			//return (2.5f * cellSize) - (cellSize - vehicle.GetWidth() - WALL_THICKNESS - MIN_CLEARANCE) * 0.5f;
			return 2.097f * cellSize;
		}
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		MAZEMAP_INLINE Smooth180ExtraLongStraight()
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);
			_instructions[3] = RelativeDirectionalDistance(Right45, 5);

		}

		virtual MAZEMAP_INLINE ManeuverCode GetManeuverID() const override { return S180ELS; }
		virtual MAZEMAP_INLINE ManeuverCode GetBackwardsManeuverID() const override { return S180ELS | MIRRORED_MANEUVER_FLAG; }
	};
	//↗2 ↖0
	/*class SingleZig : public SimpleSingleTurnManeuver<2, false, true>
	{
	public:
		SingleZig()
			: SimpleSingleTurnManeuver(0.52f, PI_F/2.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Left45, 2);
		}
	};*/
}
#endif



