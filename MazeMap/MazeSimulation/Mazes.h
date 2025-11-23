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
		static std::string maze4Data;
		static std::string singleTurnMazeData;
		static std::string APEC2016Data;
		static Maze Maze1;
		static Maze Maze2;
		static Maze Maze3;
		static Maze Maze4;
		static Maze SingleTurnMaze;
		static Maze APEC2016;

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

					int cellStart = 5 * (j + i * 16);

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
			SetupMaze(Maze4, maze4Data);
			SetupMaze(SingleTurnMaze, singleTurnMazeData);
			SetupMaze(APEC2016, APEC2016Data);

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
		static Maze& GetMaze4()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return Maze3;
		}
		static Maze& GetMazeAPEC2016()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return APEC2016;
		}
		static Maze& GetSingleTurnMaze()
		{
			if (!_setup)
			{
				SetupMazes();
			}
			return SingleTurnMaze;
		}
	};
}
