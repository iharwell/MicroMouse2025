#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverPathFinder.h"
#pragma once
#include "Defines.h"
#include "PathFinder.h"
#include "DirectionalLocation.h"
#include "HalfStepPath.h"
#include "ManeuverSet.h"
#include "ManeuverPath.h"

namespace MazeMap
{
	class ManeuverPathFinder: public PathFinder
	{
	private:
		class ScoreData
		{
		public:
			MAZEMAP_INLINE ScoreData()
				: Code(ManeuverCode::MC_NONE)
				, Cost(INFINITY)
			{ }
			MAZEMAP_INLINE ScoreData(ManeuverCode arrivalCode, float cost)
				: Code(arrivalCode)
				, Cost(cost)
			{ }
			ManeuverCode Code;
			float Cost;
		};

		DoubleMaskQueue _queue;
		ScoreData _data[31][31][8];
		float _currentBest;
		DirectionalLocation _startingPoint;
		MazeMask _deadEndMask;
	public:
		EXPORT ManeuverPathFinder(const Maze& maze, const Vehicle& vehicle);

		// Inherited via PathFinder
		EXPORT void HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE * 2>& result) override;
		EXPORT void HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result) override;
		EXPORT void HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result) override;

		EXPORT void ManeuverPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, ManeuverPath& result);
		EXPORT void ManeuverPathToGoal(CellCoordinates start, Direction startDirection, ManeuverPath& result);
		MAZEMAP_INLINE float GetLastEstimatedTime() { return _currentBest; }

		EXPORT ManeuverCode GetCode(DirectionalLocation dirLoc);
		EXPORT float GetCost(DirectionalLocation dirLoc);

	protected:
		ScoreData Score(DirectionalLocation dirLoc);
		bool UpdateScore(DirectionalLocation dirLoc, ScoreData data);
		ScoreData Score(MazeLocation loc, Direction d);
		bool UpdateScore(MazeLocation loc, Direction d, ScoreData cost);
		bool UpdateScore(MazeLocation loc, Direction d, ScoreData cost, bool suppressQueue);
		void RadiateLocation(MazeLocation loc);
		void Reset();
		void CalculateScores();

		void DescendGradient(CellCoordinates start, Direction startDirection, ManeuverPath& result);
	};
}










