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
		S1    =  1,
		S2    =  2,
		S3    =  3,
		S4    =  4,
		S5    =  5,
		S6    =  6,
		S7    =  7,
		S8    =  8,
		S9    =  9,
		S10   = 10,
		S11   = 11,
		S12   = 12,
		S13   = 13,
		S14   = 14,
		S15   = 15,
		S16   = 16,
		S17   = 17,
		S18   = 18,
		S19   = 19,
		S20   = 20,
		S21   = 21,
		S22   = 22,
		S23   = 23,
		S24   = 24,
		S25   = 25,
		S26   = 26,
		S27   = 27,
		S28   = 28,
		S29   = 29,
		S30   = 30,
		S31   = 31,

		IP45  = 32,
		IP90  = 33,
		IP135 = 34,
		IP180 = 35,
		S45SS = 36,
		S45SD = 37,
		S45LS = 38,
		S45LD = 39,
		S90SS = 40,
		S90LS = 41,
		S90SD = 42,
		S90LD = 43,
		S135SS = 44,
		S135LS = 45,
		S135SD = 46,
		S135LD = 47,
		S180SS = 48,
		S180LS = 49,
		S90ELS = 50,
		S180ELS = 51,

		MIRRORED_MANEUVER_FLAG = 0b10000000,
		
		IP45_M  = IP45  | MIRRORED_MANEUVER_FLAG,
		IP90_M  = IP90  | MIRRORED_MANEUVER_FLAG,
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
		S135SS_M = S135SS| MIRRORED_MANEUVER_FLAG,
		S135SD_M = S135SD | MIRRORED_MANEUVER_FLAG,
		S135LS_M = S135LS | MIRRORED_MANEUVER_FLAG,
		S135LD_M = S135LD | MIRRORED_MANEUVER_FLAG,
		S180SS_M = S180SS | MIRRORED_MANEUVER_FLAG,
		S180LS_M = S180LS | MIRRORED_MANEUVER_FLAG,
		S90ELS_M = S90ELS | MIRRORED_MANEUVER_FLAG,
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

	class EXPORT Maneuver
	{
		//↑↗→↘↓↙←↖
	public:
		virtual bool SupportsDiagonalEntry() const = 0;
		virtual bool SupportsStraightEntry() const = 0;

		virtual uint8_t GetStepCount() const = 0;
		virtual RelativeDirectionalDistance GetStep(uint8_t index) const = 0;

		virtual float GetCost(const Vehicle& vehicle, float cellSize) const = 0;
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const = 0;
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize) const = 0;

		virtual ManeuverCode GetManeuverID() const = 0;
		virtual ManeuverCode GetBackwardsManeuverID() const = 0;

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

	template <int SIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class EXPORT SimpleManeuver : public Maneuver
	{
	protected:
		RelativeDirectionalDistance _instructions[SIZE];
	public:
		virtual bool SupportsDiagonalEntry() const override { return DIAGONAL_ENTRY; };
		virtual bool SupportsStraightEntry() const override { return STRAIGHT_ENTRY; };

		virtual uint8_t GetStepCount() const override { return SIZE; }
		virtual RelativeDirectionalDistance GetStep(uint8_t index) const override { return _instructions[index]; }
	};

	template <int SIZE, bool STRAIGHT_ENTRY, bool DIAGONAL_ENTRY>
	class EXPORT SimpleSingleTurnManeuver : public SimpleManeuver<SIZE, STRAIGHT_ENTRY, DIAGONAL_ENTRY>
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

		virtual float GetCost(const Vehicle& vehicle, float cellSize)  const override
		{
			return 2*sqrtf( PI_F * _turnRatio/vehicle.GetMaxAngularAcceleration());
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const override { return 0.0f; }
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize)  const override { return 0.0f; }
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
	class EXPORT Smooth45ShortStraight : public ComplexSingleTurnManeuver<2,true,false>
	{
	public:
		Smooth45ShortStraight()
			: ComplexSingleTurnManeuver(0.5f * (1.0f + RT2), PI_F / 4.0f, 0.0f, 0.207f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S45SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1
	class EXPORT Smooth45ShortDiagonal : public ComplexSingleTurnManeuver<2, false, true>
	{
	public:
		Smooth45ShortDiagonal()
			: ComplexSingleTurnManeuver(0.5f * (1.0f + RT2), PI_F / 4.0f, 0.207f, 0.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S45SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 ↗1
	class EXPORT Smooth45LongStraight : public ComplexSingleTurnManeuver<2, true, false>
	{
	public:

		Smooth45LongStraight()
			: ComplexSingleTurnManeuver(1.308f, PI_F / 4.0f, 0.458f, 0.165f )
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 2);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S45LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗2
	class EXPORT Smooth45LongDiagonal : public ComplexSingleTurnManeuver<2, false, true>
	{
	public:
		Smooth45LongDiagonal()
			: ComplexSingleTurnManeuver(1.308f, PI_F / 4.0f, 0.165f, 0.458f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 2);
		}
		virtual ManeuverCode GetManeuverID() const override { return S45LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S45LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 ↗0
	class EXPORT Smooth90ShortStraight : public SimpleSingleTurnManeuver<2, true, false>
	{
	public:
		Smooth90ShortStraight()
			: SimpleSingleTurnManeuver(0.5f, PI_F / 2.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 0);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1
	class EXPORT Smooth90ShortDiagonal : public SimpleSingleTurnManeuver<2, false, true>
	{
	public:
		Smooth90ShortDiagonal()
			: SimpleSingleTurnManeuver(0.707f, PI_F / 2.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑2 →2
	class EXPORT Smooth90LongDiagonal : public SimpleSingleTurnManeuver<2, false, true>
	{
	public:
		Smooth90LongDiagonal()
			: SimpleSingleTurnManeuver(0.887f, PI_F / 2.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 2);
			_instructions[1] = RelativeDirectionalDistance(Right90, 2);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 ↗1
	class EXPORT Smooth90LongStraight : public ComplexSingleTurnManeuver<3, true, false>
	{
	public:
		Smooth90LongStraight()
			: ComplexSingleTurnManeuver(0.856f, PI_F / 2.0f, 0.144f, 0.144f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 1);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90LS | MIRRORED_MANEUVER_FLAG; }
	};


	//↑5 ↗1 ↗5
	class EXPORT Smooth90ExtraLongStraight : public ComplexSingleTurnManeuver<3, true, false>
	{
	public:
		Smooth90ExtraLongStraight()
			: ComplexSingleTurnManeuver(1.503f, 1.79245f, 1.517f, 1.517f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 5);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 5);
		}
		virtual ManeuverCode GetManeuverID() const override { return S90ELS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S90ELS | MIRRORED_MANEUVER_FLAG; }
	};

	//↑1 ↗1 →1
	class EXPORT Smooth135ShortStraight : public ComplexSingleTurnManeuver<2, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth135ShortStraight()
			: ComplexSingleTurnManeuver(0.414f, PI_F * 0.75f, 0.0f, 0.414f)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);

		}
		virtual ManeuverCode GetManeuverID() const override { return S135SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135SD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗1
	class EXPORT Smooth135ShortDiagonal : public ComplexSingleTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		Smooth135ShortDiagonal()
			: ComplexSingleTurnManeuver(0.414f, PI_F * 0.75f, 0.414f, 0.0f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 0);

		}
		virtual ManeuverCode GetManeuverID() const override { return S135SD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1
	class EXPORT Smooth135LongStraight : public ComplexSingleTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth135LongStraight()
			: ComplexSingleTurnManeuver(0.472f, PI_F*0.75f, 0.361f, 0.275f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);

		}
		virtual ManeuverCode GetManeuverID() const override { return S135LS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135LD | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 →1 ↗1
	class EXPORT Smooth135LongDiagonal : public ComplexSingleTurnManeuver<3, false, true>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.
		Smooth135LongDiagonal()
			: ComplexSingleTurnManeuver(0.472f, PI_F * 0.75f, 0.275f, 0.361f)
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 1);

		}
		virtual ManeuverCode GetManeuverID() const override { return S135LD; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S135LS | MIRRORED_MANEUVER_FLAG; }
	};

	//↗1 →1 ↗0
	class EXPORT Smooth180ShortStraight : public SimpleSingleTurnManeuver<3, true, false>
	{
	public:
		//Radius calculated to maximize distance to wall on diagonal exit of turn.
		// There's also 0.439 cells of travel on straight entry side, and 0.354 cells on diagonal exit side.

		Smooth180ShortStraight()
			: SimpleSingleTurnManeuver(0.5f, PI_F)
		{
			_instructions[0] = RelativeDirectionalDistance(Right45, 1);
			_instructions[1] = RelativeDirectionalDistance(Right90, 1);
			_instructions[2] = RelativeDirectionalDistance(Right45, 0);

		}
		virtual ManeuverCode GetManeuverID() const override { return S180SS; }
		virtual ManeuverCode GetBackwardsManeuverID() const override { return S180SS | MIRRORED_MANEUVER_FLAG; }
	};
	//↑1 ↗1 →1 ↗1
	class EXPORT Smooth180LongStraight : public SimpleManeuver<4, true, false>
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
		{
			_instructions[0] = RelativeDirectionalDistance(Forward, 1);
			_instructions[1] = RelativeDirectionalDistance(Right45, 1);
			_instructions[2] = RelativeDirectionalDistance(Right90, 1);
			_instructions[3] = RelativeDirectionalDistance(Right45, 1);

		}

		virtual float GetCost(const Vehicle& vehicle, float cellSize) const override
		{
			float vt = vehicle.GetTurnSpeed(cellSize * 0.5f);
			float a = vehicle.GetMaxForwardAcceleration();
			//float entryCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _entryLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			//float exitCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _exitLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			float entryExitCost = LinearKinematics::TIgnoringV1(GetEntryDistance(vehicle, cellSize), vt, a);
			return RotationalKinematics::TGivenCentVtTheta(vehicle.GetMaxLateralAcceleration(), vt, PI_F) + entryExitCost + entryExitCost;
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * 0.5f);
			float a = vehicle.GetMaxForwardAcceleration();
			return LinearKinematics::V1IgnoringT(GetEntryDistance(vehicle, cellSize) * cellSize, turnSpeed, a);

		}
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * 0.5f);
			float a = vehicle.GetMaxForwardAcceleration();
			return LinearKinematics::V1IgnoringT(GetEntryDistance(vehicle, cellSize) * cellSize, turnSpeed, a);
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

		virtual float GetCost(const Vehicle& vehicle, float cellSize) const override
		{
			float vt = vehicle.GetTurnSpeed(cellSize * 0.678f);
			float a = vehicle.GetMaxForwardAcceleration();
			//float entryCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _entryLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			//float exitCost = (-vt + sqrtf(vt * vt + 2.0f * vehicle.GetMaxForwardAcceleration() * _exitLength * cellSize)) / vehicle.GetMaxForwardAcceleration();
			float entryExitCost = LinearKinematics::TIgnoringV1(GetEntryDistance(vehicle, cellSize), vt, a);
			return RotationalKinematics::TGivenCentVtTheta(vehicle.GetMaxLateralAcceleration(), vt, PI_F) + entryExitCost + entryExitCost;
		}
		virtual float GetEntrySpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * 0.678f);
			float a = vehicle.GetMaxForwardAcceleration();
			return LinearKinematics::V1IgnoringT(GetEntryDistance(vehicle, cellSize) * cellSize, turnSpeed, a);

		}
		virtual float GetExitSpeed(const Vehicle& vehicle, float cellSize) const override
		{
			float turnSpeed = vehicle.GetTurnSpeed(cellSize * 0.678f);
			float a = vehicle.GetMaxForwardAcceleration();
			return LinearKinematics::V1IgnoringT(GetEntryDistance(vehicle, cellSize) * cellSize, turnSpeed, a);
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
