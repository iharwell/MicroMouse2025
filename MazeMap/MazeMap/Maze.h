#ifndef MAZE_H
#define MAZE_H
#define MAZE_EXPORT
#include "Defines.h"
#include "CellCoordinates.h"
#include "MazeLocation.h"
#include "DirectionalLocation.h"
#include "Cell.h"
#include "MazeMask.h"

#ifdef MAZE_EXPORT
#include <sstream>
#include <fstream>
#endif

namespace MazeMap
{
	class EXPORT Maze
	{
	private:
		Cell _cells[16][16];
		float _cellDimension;
		CellCoordinates _goal;
		MazeMask _reachable;
		bool _complete;
		bool _reachableCalculated;
		bool _goalFound;
		DoubleMazeMask _accessible;
		void FindReachables(MazeMask& mask, CellCoordinates cell) const;
	public:
		Maze();
		Maze(float cellDimension);

		uint8_t GetXSize();
		uint8_t GetXSize() const;
		uint8_t GetYSize();
		uint8_t GetYSize() const;

		Cell& operator()(uint8_t x, uint8_t y);
		const Cell&  operator()(uint8_t x, uint8_t y) const;
		Cell& operator[](CellCoordinates coords);
		const Cell& operator[](CellCoordinates coords) const;

		void SetWall(Cell& cell, Direction direction, WallState state);

		Cell& Index(int x, int y);
		const Cell& Index(int x, int y) const;
		Cell& Index(CellCoordinates coords);
		const Cell& Index(CellCoordinates coords) const;

		float GetCellDimension();
		float GetCellDimension() const;

		bool HasFoundGoal();

		MazeMask GetReachableMask() const;
		MazeMask GetReachableMask();

		CellCoordinates GetGoalLowerLeft();
		CellCoordinates GetGoalLowerLeft() const;

		bool IsComplete();

		void PreCalculate();

		bool IsAccessibleLocation(MazeLocation location);
		bool IsAccessibleLocation(MazeLocation location) const;

		bool IsIntersection(CellCoordinates location);
		bool IsIntersection(CellCoordinates location) const;

		bool IsIntersection(MazeLocation location);
		bool IsIntersection(MazeLocation location) const;

		MazeMask DeadEndMask(CellCoordinates startLocation) const;

		bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction);
		bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction) const;
#ifdef MAZE_EXPORT

		void ExportToFile(std::wstring fileName)
		{
			std::ofstream file(fileName);
			for (uint8_t i = 0; i < 16; ++i)
			{
				file << "\"";
				for (uint8_t j = 0; j < 16; ++j)
				{
					CharBlock cb = Index(j, i).Serialize();
					file << cb.chars[0] << cb.chars[1] << cb.chars[2] << cb.chars[3];
					if (j == 15)
					{
						file << "\\n\"" << std::endl;
					}
					else
					{
						file << ",";
					}
				}
			}//apec2010.txt
			file.flush();
			file.flush();
			file.close();
		}
		void ExportToFileC(std::wstring fileName) const
		{
			std::ofstream file(fileName);
			for (uint8_t i = 0; i < 16; ++i)
			{
				file << "\"";
				for (uint8_t j = 0; j < 16; ++j)
				{
					CharBlock cb = Index(j, i).Serialize();
					file << cb.chars[0] << cb.chars[1] << cb.chars[2] << cb.chars[3];
					if (j == 15)
					{
						file << "\\n\"" << std::endl;
					}
					else
					{
						file << ",";
					}
				}
			}//apec2010.txt
			file.flush();
			file.flush();
			file.close();
		}

		void DumpMaze() const
		{
			ExportToFileC(L"mazeDump.txt");
		}
#endif
	};
}

#endif