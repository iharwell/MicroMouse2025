#pragma once
#include "..\MazeMap\Maze.h"
#include <string>
namespace MazeMap
{
	class Mazes
	{
	private:
		static bool _setup;
		static std::string maze1Data;
		static std::string maze2Data;
		static std::string maze3Data;
		static Maze Maze1;
		static Maze Maze2;
		static Maze Maze3;

		static WallState CharToWall(char c)
		{
			if (c == 'O' || c == 'o')
			{
				return WallState::NoWall;
			}
			else if (c == 'W' || c == 'w')
			{
				return WallState::Wall;
			}
			return WallState::Unknown;
		};

		static void SetupMaze(Maze& maze, const std::string& mazeData)
		{
			maze = Maze();
			for (int i = 0; i < 16; ++i)
			{
				for (int j = 0; j < 16; ++j)
				{
					Cell& c = maze((uint8_t)j, (uint8_t)i);

					int cellStart = 5 * (j+i*16);

					char up = mazeData[cellStart];
					char down = mazeData[cellStart + 1];
					char left = mazeData[cellStart + 2];
					char right = mazeData[cellStart + 3];

					c.SetUp(CharToWall(up));
					c.SetDown(CharToWall(down));
					c.SetLeft(CharToWall(left));
					c.SetRight(CharToWall(right));
				}
			}
			if (maze.IsComplete())
			{
				maze.PreCalculate();
			}
		}
	public:
		static void SetupMazes()
		{
			SetupMaze(Maze1, maze1Data);
			SetupMaze(Maze2, maze2Data);
			SetupMaze(Maze3, maze3Data);

			_setup = true;
		}

		static Maze& GetMaze1()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return Maze1;
		}
		static Maze& GetMaze2()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return Maze2;
		}
		static Maze& GetMaze3()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return Maze3;
		}
	};
}