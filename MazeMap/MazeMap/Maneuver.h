#ifndef MANEUVER_H
#define MANEUVER_H
#include "Defines.h"
#include "MazeLocation.h"
#include "DirectionalLocation.h"
#include "Maze.h"
#include "Vehicle.h"
#include "Kinematics.h"

namespace MazeMap
{
	enum ManeuverCode : uint8_t
	{
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
	inline constexpr int8_t CodeDegrees(ManeuverCode mc)
	{
		if (mc < 32)
		{
			return 0;
		}

		int8_t mult = 1 - 2 * static_cast<uint8_t>(mc & MIRRORED_MANEUVER_FLAG);

		auto normCode = mc & INVERTED_MIRRORED_MANEUVER_FLAG;

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
		}
	}

	class EXPORT Maneuver
	{
		//↑↗→↘↓↙←↖
	public:
		virtual bool SupportsDiagonalEntry() const = 0; //
		virtual bool SupportsStraightEntry() const = 0; //

		virtual uint8_t GetStepCount() const = 0; //
		virtual RelativeDirectionalDistance GetStep(uint8_t index) const = 0; //

		virtual float GetCost(const Vehicle& vehicle) const = 0; //
		virtual float GetEntrySpeed(const Vehicle& vehicle) const = 0; //
		virtual float GetExitSpeed(const Vehicle& vehicle) const = 0; //

		virtual ManeuverCode GetManeuverID() const = 0;
		virtual ManeuverCode GetBackwardsManeuverID() const = 0;

		virtual float GetVMax(const Vehicle& vehicle) const = 0; //
		//virtual float GetTurnRadius(const Vehicle& vehicle) const = 0;
		//virtual float GetTurnInLength(const Vehicle& vehicle) const = 0;


		bool IsValidMove(DirectionalLocation start, const Maze& maze, bool mirrored) const
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

		DirectionalLocation Move(DirectionalLocation start, bool mirrored) const
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

		uint8_t DistanceTravelled() const
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
	class EXPORT SimpleManeuver : public Maneuver
	{
	protected:
		RelativeDirectionalDistance _instructions[STEPSIZE];
	public:
		virtual bool SupportsDiagonalEntry() const override { return DIAGONAL_ENTRY; };
		virtual bool SupportsStraightEntry() const override { return STRAIGHT_ENTRY; };

		virtual uint8_t GetStepCount() const override { return STEPSIZE; }
		virtual RelativeDirectionalDistance GetStep(uint8_t index) const override { return _instructions[index]; }
	};

	template <int STEPSIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class EXPORT SmoothTurnManeuver : public SimpleManeuver<STEPSIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
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
		SmoothTurnManeuver(float radius_in_cells, float radians, float turnInDelta, float preTurnDist, float postTurnDist)
			: _radius_in_cells(radius_in_cells)
			, _radians(radians)
			, _turnInDelta(turnInDelta)
			, _preTurnDist(preTurnDist)
			, _postTurnDist(postTurnDist)
		{
		}

		float GetArcLengthInCells()
		{
			//float angleChangeInDeltas = _turnInDelta / _radius_in_cells;
			//float arcRadians = _radians - angleChangeInDeltas;
			//float arcDist = arcRadians * _radius_in_cells;

			return _radians * _radius_in_cells - _turnInDelta;
		}
	public:
		float GetRadians() const { return _radians; }
		float GetRadiusInCells() const { return _radius_in_cells; }
		float GetTurnInDistInCells() const { return _turnInDelta; }
		float GetPreTurnDistInCells() const { return _preTurnDist; }
		float GetPostTurnDistInCells() const { return _postTurnDist; }

		float GetTravelDistInCells() const
		{
			//return _preTurnDist + 2.0f * _turnInDelta + GetArcLengthInCells() + _postTurnDist;
			return _preTurnDist + _turnInDelta + _radians * _radius_in_cells + _postTurnDist;
		}
        virtual float GetVMax(const Vehicle& vehicle) const override
        {
            const float cellSize = Maze::GetCellDimension() / 100.0f;
            return vehicle.GetTurnSpeed(GetRadiusInCells() * cellSize);
        }
        virtual float GetCost(const Vehicle& vehicle) const override
        {
            const float cellSize = Maze::GetCellDimension() / 100.0f;
            float turnSpeed = vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
            float turnCost = cellSize * GetTravelDistInCells() / turnSpeed;
            return turnCost;
        }
		virtual float GetEntrySpeed(const Vehicle& vehicle) const override
		{
			//const float cellSize = Maze::GetCellDimension();
			//return vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
			return GetVMax(vehicle);
		}
		virtual float GetExitSpeed(const Vehicle& vehicle) const override
		{
			//const float cellSize = Maze::GetCellDimension();
			//return vehicle.GetTurnSpeed(cellSize * _radius_in_cells);
			return GetVMax(vehicle);
		}
	};
	/*
	template <int STEPSIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class EXPORT SimpleSingleTurnManeuver : public SimpleManeuver<STEPSIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
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
		float GetRadians() const { return _radians; }
		float GetRadiusInCells() const { return _radius_in_cells; }

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
	class EXPORT ComplexSingleTurnManeuver : public SimpleSingleTurnManeuver<SIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
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
	class EXPORT TurnInPlace : public SimpleManeuver<1, true, true>
	{
		const float _turnRatio;
	public:
		TurnInPlace( float pisTurned, RelativeDirectionalDistance instruction)
			: _turnRatio(pisTurned)
		{
			_instructions[0] = instruction;
		}

		virtual float GetVMax(const Vehicle& vehicle) const override
		{
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

		virtual float GetCost(const Vehicle& vehicle)  const override
		{
			return 2*sqrtf( PI_F * _turnRatio/vehicle.GetMaxAngularAcceleration());
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle) const override { return 0.0f; }
		virtual float GetExitSpeed(const Vehicle& vehicle)  const override { return 0.0f; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return GetManeuverID() | MIRRORED_MANEUVER_FLAG; }
	};

	//↗0
	class EXPORT TurnInPlace45 : public TurnInPlace
	{
	public:
		TurnInPlace45()
			: TurnInPlace(0.25f, RelativeDirectionalDistance(Right45, 0))
		{}
		virtual ManeuverCode GetManeuverID() const override { return IP45; }
	};
	//→0
	class EXPORT TurnInPlace90 : public TurnInPlace
	{
	public:
		TurnInPlace90()
			: TurnInPlace(0.5f, RelativeDirectionalDistance(Right90, 0))
		{
		}

		virtual ManeuverCode GetManeuverID() const override { return IP90; }
	};
	//↘0
	class EXPORT TurnInPlace135 : public TurnInPlace
	{
	public:
		TurnInPlace135()
			: TurnInPlace(0.75f, RelativeDirectionalDistance(Right135, 0))
		{}

		virtual ManeuverCode GetManeuverID() const override { return IP135; }
	};
	//↓0
	class EXPORT TurnInPlace180 : public TurnInPlace
	{
	public:
		TurnInPlace180()
			: TurnInPlace(1.0f, RelativeDirectionalDistance(Reverse, 0))
		{
		}

		virtual ManeuverCode GetManeuverID() const override { return IP180; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return IP180; }
	};


	//↑1 ↗1
	class EXPORT Smooth45ShortStraight : public SmoothTurnManeuver<2,true,false>
	{
	public:
		Smooth45ShortStraight()
			: SmoothTurnManeuver(156/180.0f, PI_F / 4.0f, 32/180.0f, 9/180.0f,42.3f/180.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S45SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45SD | MIRRORED_MANEUVER_FLAG; }
	};

	//↑1 ↗1
	class EXPORT Smooth45ShortDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		Smooth45ShortDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S45SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 ↗1
	class EXPORT Smooth45LongStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:

		Smooth45LongStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S45LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗2
	class EXPORT Smooth45LongDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		Smooth45LongDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S45LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 ↗0
	class EXPORT Smooth90ShortStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:
		Smooth90ShortStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S90SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1
	class EXPORT Smooth90ShortDiagonal : public SmoothTurnManeuver<2, false, true>
	{
	public:
		Smooth90ShortDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S90SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 →2
	/*class EXPORT Smooth90LongDiagonal : public SmoothTurnManeuver<2, false, true>
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
		virtual ManeuverCode GetManeuverID() const override { return S90LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90LD | MIRRORED_MANEUVER_FLAG; }
	};*/
	//↑1 ↗1 ↗1
	class EXPORT Smooth90LongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		Smooth90LongStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S90LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90LS | MIRRORED_MANEUVER_FLAG; }
	};


	/*//↑5 ↗1 ↗5
	class EXPORT Smooth90ExtraLongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		Smooth90ExtraLongStraight()
			: SmoothTurnManeuver(1.503f, 1.79245f, 1.517f, 1.517f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 5);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90ELS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90ELS | MIRRORED_MANEUVER_FLAG; }
	};*/
	//↑1 ↗2 ↗1
	class EXPORT Smooth90ExtraLongDiagonal : public SmoothTurnManeuver<3, true, false>
	{
	public:
		Smooth90ExtraLongDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S90ELD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90ELD | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 →1
	class EXPORT Smooth135ShortStraight : public SmoothTurnManeuver<2, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth135ShortStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S135SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗0
	class EXPORT Smooth135ShortDiagonal : public SmoothTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		Smooth135ShortDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S135SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1
	class EXPORT Smooth135LongStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth135LongStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S135LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗1
	class EXPORT Smooth135LongDiagonal : public SmoothTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		Smooth135LongDiagonal()
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
		virtual ManeuverCode GetManeuverID() const override { return S135LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 →1 ↗0
	class EXPORT Smooth180ShortStraight : public SmoothTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth180ShortStraight()
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
		virtual ManeuverCode GetManeuverID() const override { return S180SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S180SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1 ↗1
	class EXPORT Smooth180LongStraight : public SmoothTurnManeuver<4, true, false>
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

		Smooth180LongStraight()
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

		virtual ManeuverCode GetManeuverID() const override { return S180LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S180LS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑5 ↗1 →1 ↗5
	class EXPORT Smooth180ExtraLongStraight : public SimpleManeuver<4, true, false>
	{
	private:
		float GetLeadingDistance(const Vehicle& vehicle, float cellSize) const
		{
			return (cellSize - vehicle.GetWidth() - WALL_THICKNESS - MIN_CLEARANCE) * 0.75f;
		}
		float GetEntryDistance(const Vehicle& vehicle, float cellSize) const
		{
			//return (2.5f * cellSize) - (cellSize - vehicle.GetWidth() - WALL_THICKNESS - MIN_CLEARANCE) * 0.5f;
			return 2.097f * cellSize;
		}
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth180ExtraLongStraight()
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);
			_instructions[3] = RelativeDirectionalDistance(Right45, 5);

		}

		virtual ManeuverCode GetManeuverID() const override { return S180ELS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S180ELS | MIRRORED_MANEUVER_FLAG; }
	};
	//↗2 ↖0
	/*class EXPORT SingleZig : public SimpleSingleTurnManeuver<2, false, true>
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

