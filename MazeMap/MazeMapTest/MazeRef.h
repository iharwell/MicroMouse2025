#pragma once
#include "..\MazeMap\Maze.h"
#include <string>
namespace MazeMap
{
	static class Mazes
	{
	private:
		static bool _setup;
		static std::string maze1Data;
		static Maze Maze1;

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
		}
	public:
		static void SetupMazes()
		{
			SetupMaze(Maze1, maze1Data);

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
	};
	bool Mazes::_setup = false;
	Maze Mazes::Maze1 = Maze();
	std::string Mazes::maze1Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW,WWWO,OWOO,OWOO,OWOW,OWWO,OWOO,OWOO,OWOW\n"
		"OOWW,WOWO,OWOO,OWOO,OWOO,OWOW,OWWW,OOWO,WWOO,OOOW,OOWW,OOWW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WOOW,WOWW,OOWO,WWOO,WOOW,WOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WWOO,WWOO,WWOO,WOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,OWOW,WWWO,WWOO,WWOO,WWOO,WWOO,OWOO,OOOW\n"
		"OOWO,WWOO,OOOO,OOOW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,OWOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,OWOW,OWWW,OOWO,WWOO,OWOW,OWWW,OOWW\n"
		"OOWW,OWWO,WOOO,WOOO,WOOO,WOOW,WOWW,OOWO,WWOO,OOOW,OOWW,OOWW,OWWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,WWWO,WOOO,WOOO,WOOW,WOWO,WOOO,WOOO,WOOW");
}