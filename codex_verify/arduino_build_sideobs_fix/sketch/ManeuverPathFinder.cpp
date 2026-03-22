#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverPathFinder.cpp"
#include "pch.h"
#include "ManeuverPathFinder.h"

namespace MazeMap
{
	namespace
	{
		inline bool IsTrackedLocation(MazeLocation loc)
		{
			return loc.GetX() > 0 && loc.GetX() < 32 && loc.GetY() > 0 && loc.GetY() < 32;
		}
	}

	ManeuverPathFinder::ManeuverPathFinder(const Maze& maze, const Vehicle& vehicle)
		: PathFinder(maze, vehicle)
		, _data()
		, _queue()
		, _currentBest()
		, _startingPoint()
		, _deadEndMask()
	{
	}
	void ManeuverPathFinder::HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE * 2>& result)
	{
	}

	void ManeuverPathFinder::HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
	}
	void ManeuverPathFinder::ManeuverPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, ManeuverPath& result)
	{
		_deadEndMask = GetMaze().DeadEndMask(start);
		Reset();
		result.clear();
		for (uint8_t i = 0; i < 8; i++)
		{
			UpdateScore(DirectionalLocation(MazeLocation::CellCenter(end), OrdinalDirections[i]), ScoreData(MC_NONE, 0.0f));
		}

		CalculateScores();
		DescendGradient(start, startDirection, result);
		_currentBest = result.Cost(GetVehicle(), GetMaze().GetCellDimension() / 100.0f);
	}
	void ManeuverPathFinder::ManeuverPathToGoal(CellCoordinates start, Direction startDirection, ManeuverPath& result)
	{
		_deadEndMask = GetMaze().DeadEndMask(start);
		Reset();
		result.clear();
		MazeLocation goalLLC = MazeLocation::CellCenter(GetMaze().GetGoalLowerLeft());
		for (uint8_t i = 0; i < 3; i += 2)
		{
			for (uint8_t j = 0; j < 3; j += 2)
			{
				if (i == 1 && j == 1)
				{
					continue;
				}
				for (uint8_t k = 0; k < 8; k++)
				{
					DirectionalLocation dl(MazeLocation(goalLLC.GetX() + i, goalLLC.GetY() + j), OrdinalDirections[k]);
					UpdateScore(dl, ScoreData(ManeuverCode::MC_NONE, 0.0f));
				}
			}
		}
		CalculateScores();
		DescendGradient(start, startDirection, result);
		_currentBest = result.Cost(GetVehicle(), GetMaze().GetCellDimension() / 100.0f);
	}

	float ManeuverPathFinder::GetCost(DirectionalLocation dirLoc)
	{
		return Score(dirLoc).Cost;
	}
	ManeuverCode ManeuverPathFinder::GetCode(DirectionalLocation dirLoc)
	{
		return Score(dirLoc).Code;
	}

	void ManeuverPathFinder::HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		Reset();
		result.clear();
		ManeuverPath* p = new ManeuverPath();
		ManeuverPathToGoal(start, startDirection, *p);
		_currentBest = p->Cost(GetVehicle(), GetMaze().GetCellDimension() / 100.0f);
		p->ToHalfStepPath(DirectionalLocation(MazeLocation::CellCenter(start), startDirection), result);
		delete p;
	}

	void ManeuverPathFinder::CalculateScores()
	{
		ManeuverSet& ms = ManeuverSet::GetSet();
		const Vehicle& v = GetVehicle();
		const Maze& m = GetMaze();
		float cellDim = GetMaze().GetCellDimension() / 100;
		ms.SortByCost(GetVehicle(), cellDim);
		_queue.SwapQueues();
		while (_queue.GetCurrent().AnyFlags())
		{
			for (uint8_t i = 1; i < 32; i++)
			{
				if (!_queue.GetCurrent().GetRow(i))
				{
					continue;
				}
				for (uint8_t j = 1; j < 32; j++)
				{
					if (_queue.GetCurrent(i, j))
					{
						RadiateLocation(MazeLocation(i, j));
					}
				}
			}
			_queue.SwapQueues();
		}
	}

	void ManeuverPathFinder::DescendGradient(CellCoordinates start, Direction startDirection, ManeuverPath& result)
	{
		const ManeuverSet& ms = ManeuverSet::GetSet();
		const Maze& m = GetMaze();
		ScoreData currentScore = Score(MazeLocation::CellCenter(start), -startDirection);
		DirectionalLocation current(MazeLocation::CellCenter(start), startDirection);

		while (currentScore.Code != MC_NONE && currentScore.Cost > 0.0001)
		{
			if (currentScore.Code <= ManeuverCode::S31)
			{
				result.push_back(currentScore.Code);
				current = current.MoveForward(currentScore.Code);
				currentScore = Score(current.GetLocation(), -current.GetDirection());
			}
			else
			{
				ManeuverCode effectiveCode = ms[currentScore.Code].GetBackwardsManeuverID();
				if (currentScore.Code & ManeuverCode::MIRRORED_MANEUVER_FLAG)
				{
					effectiveCode = effectiveCode ^ ManeuverCode::MIRRORED_MANEUVER_FLAG;
				}
				result.push_back(effectiveCode);
				const Maneuver& man = ms[effectiveCode];
#ifdef _WINDOWS
				if (!man.IsValidMove(current, m, effectiveCode & MIRRORED_MANEUVER_FLAG))
				{
					throw std::errc::bad_message;
				}
#endif
				current = man.Move(current, effectiveCode & MIRRORED_MANEUVER_FLAG);
				currentScore = Score(current.GetLocation(), -current.GetDirection());
			}
		}
	}

	ManeuverPathFinder::ScoreData ManeuverPathFinder::Score(DirectionalLocation dirLoc)
	{
		return Score(dirLoc.GetLocation(), dirLoc.GetDirection());
	}
	ManeuverPathFinder::ScoreData ManeuverPathFinder::Score(MazeLocation loc, Direction d)
	{
		if (!IsTrackedLocation(loc))
		{
			return ScoreData();
		}
		return _data[loc.GetX() - 1][loc.GetY() - 1][RelativeDirections[d]];
	}
	bool ManeuverPathFinder::UpdateScore(DirectionalLocation dirLoc, ScoreData data)
	{
		return UpdateScore(dirLoc.GetLocation(), dirLoc.GetDirection(), data);
	}
	bool ManeuverPathFinder::UpdateScore(MazeLocation loc, Direction d, ScoreData cost)
	{
		if (!IsTrackedLocation(loc))
		{
			return false;
		}
		ScoreData prev = _data[loc.GetX() - 1][loc.GetY() - 1][RelativeDirections[d]];
		if (prev.Cost > cost.Cost)
		{
			_queue.Enqueue(loc);
			_data[loc.GetX() - 1][loc.GetY() - 1][RelativeDirections[d]] = cost;
			return true;
		}
		return false;
	}
	bool ManeuverPathFinder::UpdateScore(MazeLocation loc, Direction d, ScoreData cost, bool suppressQueue)
	{
		if (!IsTrackedLocation(loc))
		{
			return false;
		}
		ScoreData prev = _data[loc.GetX() - 1][loc.GetY() - 1][RelativeDirections[d]];
		if (prev.Cost > cost.Cost)
		{
			if (!suppressQueue)
			{
				_queue.Enqueue(loc);
			}
			_data[loc.GetX() - 1][loc.GetY() - 1][RelativeDirections[d]] = cost;
			return true;
		}
		return false;
	}
	void ManeuverPathFinder::RadiateLocation(MazeLocation loc)
	{
		int TIP45Count = 0;
		ManeuverSet& ms = ManeuverSet::GetSet();
		float cellDim = GetMaze().GetCellDimension() / 100;
		const Vehicle& v = GetVehicle();
		const Maze& m = GetMaze();
		for (uint8_t i = 0; i < 8; i++)
		{
			Direction fromDir = OrdinalDirections[i];
			ScoreData fromVal = Score(loc, fromDir);
			bool diag = IsDiagonal(fromDir);
			float entrySpeed = 0.0f;
			if (fromVal.Code != MC_NONE && fromVal.Code > S31)
			{
				entrySpeed = ms[fromVal.Code].GetExitSpeed(v);
			}
			if (!isfinite(fromVal.Cost))
			{
				continue;
			}
			MazeLocation currentLoc = loc;
			float distance = 0.0f;
			uint8_t straightDistance = 0;
			while (IsTrackedLocation(currentLoc)
				&& m.IsAccessibleLocation(currentLoc)
				&& !_deadEndMask.GetFlag(currentLoc.GetX() >> 1, currentLoc.GetY() >> 1)
				&& !_deadEndMask.GetFlag((currentLoc.GetX() - 1) >> 1, (currentLoc.GetY() - 1) >> 1))
			{
				for (uint8_t j = 0; j < ms.size(); j++)
				{
					const Maneuver& man = ms[j];

					if (man.DistanceTravelled() == 0)
					{
						continue;
					}

					bool direct = man.IsValidMove(DirectionalLocation(currentLoc, fromDir), m, false);
					bool mirrored = man.IsValidMove(DirectionalLocation(currentLoc, fromDir), m, true);

					if (!(direct || mirrored))
					{
						continue;
					}
					float manCost = man.GetCost(v);
					float straightCost = 0.0f;
					if (straightDistance > 0)
					{
						straightCost = v.GetStraightLineCost(distance, entrySpeed, man.GetEntrySpeed(v));
						if (straightCost < 0.0f)
						{
							continue;
						}
						if (Score(currentLoc, fromDir).Cost > (fromVal.Cost + straightCost))
						{
							UpdateScore(currentLoc, fromDir, ScoreData(static_cast<ManeuverCode>(straightDistance), fromVal.Cost + straightCost), true);
						}
					}
					float postManCost = fromVal.Cost + straightCost + manCost;
					if (direct)
					{
						DirectionalLocation current = DirectionalLocation(currentLoc, fromDir);
						current = man.Move(current, false);
						if (Score(current).Cost > postManCost)
						{
							UpdateScore(current, ScoreData(man.GetManeuverID(), postManCost));
						}
					}
					if (mirrored)
					{
						DirectionalLocation current = DirectionalLocation(currentLoc, fromDir);
						current = man.Move(current, true);
						ScoreData currentScore = Score(current);
						if (currentScore.Cost > postManCost)
						{
							UpdateScore(current, ScoreData(man.GetManeuverID() | ManeuverCode::MIRRORED_MANEUVER_FLAG, postManCost));
						}
					}
				}

				ScoreData currentScore = Score(currentLoc, fromDir);
				float straightCost = v.GetStraightLineCost(distance, entrySpeed, 0.0f);
				UpdateScore(currentLoc, fromDir, ScoreData(static_cast<ManeuverCode>(straightDistance), fromVal.Cost + straightCost), true);
				if (diag)
				{
					distance += GetMaze().GetCellDimension() / 100 * sqrtf(2.0f);
				}
				else
				{
					distance += GetMaze().GetCellDimension() / 100;
				}
				currentLoc = currentLoc >> fromDir;
				++straightDistance;
			}
		}
	}
	void ManeuverPathFinder::Reset()
	{
		ScoreData defScore = ScoreData();
		for (size_t i = 0; i < 31; i++)
		{
			for (size_t j = 0; j < 31; j++)
			{
				for (size_t k = 0; k < 8; k++)
				{
					_data[i][j][k] = defScore;
				}
			}
		}
		_queue.Clear();
	}
}




