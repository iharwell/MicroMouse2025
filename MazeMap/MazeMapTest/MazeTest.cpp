#include "pch.h"
#define MAZE_EXPORT
#include "CppUnitTest.h"
#include "..\MazeMap\Maze.h"
#include "MazeRef.h"
#include <vector>

#include <sstream>
#include <chrono>
#include "Templates.h"
#include "..\MazeMap\PathFinder.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace MazeMap
{
	std::wstring sampleName = std::wstring(L"sampleMaze.txt");

	bool Mazes::_setup = false;
	Maze Mazes::Maze1 = Maze();
	Maze Mazes::Maze2 = Maze();
	Maze Mazes::Maze3 = Maze();
	Maze Mazes::Maze4 = Maze();
	Maze Mazes::SingleTurnMaze = Maze();
	Maze Mazes::APEC2016 = Maze();
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
	std::string Mazes::maze2Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW,WWWO,OWOO,OWOO,OWOW,OWWO,OWOO,OWOO,OWOW\n"
		"OOWW,WOWO,OWOO,OWOO,OWOO,OWOW,OWWW,OOWO,WWOO,OOOW,OOWW,OOWW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WOOW,WOWW,OOWO,WWOO,WOOW,WOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOW,UWWU,UWUW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WWOO,WWOO,WWOO,WOOO,WWOW,WUWU,WUUW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,OWOW,WWWO,WWOO,WWOO,WWOO,WWOO,OWOO,OOOW\n"
		"OOWO,WWOO,OOOO,OOOW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,OWOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,OWOW,OWWW,OOWO,WWOO,OWOW,OWWW,OOWW\n"
		"OOWW,OWWO,WOOO,WOOO,WOOO,WOOW,WOWW,OOWO,WWOO,OOOW,OOWW,OOWW,OWWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,WWWO,WOOO,WOOO,WOOW,WOWO,WOOO,WOOO,WOOW");
	std::string Mazes::maze3Data = std::string(
		"OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"OOWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW\n"
		"OOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"WOWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OOOW\n"
		"OWWW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,OOWW\n"
		"OOWW,OWWW,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOW,OWWO,OWOW,WWWO,WWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WOOW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWOO,OWOO,OWOO,OWOO,OWOW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW\n"
	);
	std::string Mazes::maze4Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOO,WWOW,OWWW,OWWO,OWOW,OWWW,OWWW,OOWW\n"
		"OOWO,WOOW,WOWO,WWOO,WWOO,WWOO,OWOW,OWWO,WOOW,OWWO,OOOO,WOOW,OOWO,OOOO,OOOW,OOWW\n"
		"OOWW,OWWO,OWOW,OWWW,OWWW,WWWO,OOOW,WOWO,WWOO,WOOW,WOWW,OWWW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,WWOW,WOWO,OWOW,OWWO,WWOO,WWOO,OOOO,WWOW,WOWW,OOWW,OOWW\n"
		"OOWW,OOWW,WOWO,WOOW,OOWW,OWWO,OWOW,WOWO,WOOW,OWWO,WWOO,WOOW,OWWO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OWWO,OWOW,WOWO,WOOW,OOWO,OWOO,WWOW,OOWW,OWWO,OWOW,OOWW,OWWW,WOWW,OOWW\n"
		"OOWW,OOWW,OOWW,WOWO,OWOO,OWOW,OOWW,OOWO,OWOW,OOWW,OOWW,OOWW,WOWO,OOOO,WWOW,OOWW\n"
		"OOWW,OOWO,OOOO,WWOW,WOWW,OOWW,OOWW,WOWO,WOOW,OOWW,OOWW,WOWO,OWOW,WOWO,OWOW,OOWW\n"
		"OOWW,WOWW,WOWW,OWWO,OWOW,OOWW,WOWO,WWOO,WWOO,WOOW,WOWO,OWOW,WOWO,WWOO,WOOW,OOWW\n"
		"OOWW,OWWO,WWOO,WOOW,WOWO,WOOW,OWWO,WWOO,WWOO,OWOO,WWOW,OOWW,OWWO,OWOW,OWWW,OOWW\n"
		"OOWW,OOWW,WWWO,OWOO,WWOO,OWOW,WOWO,OWOW,OWWO,WOOW,OWWW,WOWO,WOOW,WOWO,OOOO,OOOW\n"
		"OOWW,WOWO,OWOW,OOWW,OWWO,WOOW,OWWW,OOWW,OOWW,WWWO,OOOO,WWOO,OWOW,WWWO,WOOW,WOWW\n"
		"OOWW,OWWO,WOOW,OOWW,OOWW,WWWO,OOOO,WOOW,OOWW,OWWO,WOOW,OWWO,WOOO,WWOW,OWWW,OWWW\n"
		"OOWW,WOWO,WWOO,WOOW,WOWO,WWOO,WOOW,WWWO,WOOO,WOOW,WWWO,WOOO,WWOO,WWOO,OOOO,WOOW\n"
		"WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOO,WWOW\n"
		);
	std::string Mazes::singleTurnMazeData = std::string(
		"OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW,OWWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,WOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OWWO,OWOW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"WOWO,WOOO,WOOW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW\n"
	);
	std::string Mazes::APEC2016Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWO,WOOW,OOWW,OWWO,WWOO,WWOO,OWOO,WWOO,OWOO,WWOW,OWWO,OWOW,OWWO,OWOW,OOWW,OOWW\n"
		"OOWW,OWWO,OOOW,WOWO,WWOO,OWOW,WOWO,OWOO,WOOO,OWOW,OOWW,WOWO,WOOW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OWWO,OWOW,OOWW,OWWO,WOOO,OWOO,WOOW,OOWW,OWWO,WWOO,WOOW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWO,WOOW,WOWO,WWOO,WOOO,OWOW,OOWW,OOWW,OWWO,WWOO,WOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,WOWO,OWOW,OWWW,WWWO,WWOO,WOOO,WOOW,OOWW,OOWW,OWWW,OWWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OWWW,WOWO,OOOW,OWWO,OWOO,OWOO,WWOO,WOOW,OOWO,OOOW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,WOWO,WOOO,OWOW,OOWW,WOWO,WOOW,WOWO,OWOO,WWOW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OWWO,OWOW,OOWW,OOWO,WWOO,OWOW,WWWO,WOOO,OWOO,WWOW,OOWO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,WOWO,WOOW,WOWW,OWWW,OOWW,OWWO,OWOW,WOWO,OWOW,WOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOW,OOWW,OOWW,OOWW,WWWO,OOOO,OWOW,WOWO,OOOW\n"
		"OOWW,OOWW,OOWW,WOWW,OWWO,WWOO,OWOW,OOWW,OOWW,OOWW,OOWW,WWWO,WOOW,WOWO,OWOW,OOWW\n"
		"OOWW,OOWW,WOWO,WWOO,WOOO,WWOW,WOWO,WOOW,WOWO,WOOW,WOWO,WWOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OOOW\n"
		"WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW\n"
	);

	TEST_CLASS(MazeTest)
	{
	public:



		static void ValidateMaze(const Maze& maze)
		{
			bool error = false;
			for (int i = 0; i < maze.GetXSize(); i++)
			{
				for (int j = 0; j < maze.GetYSize(); j++)
				{
					Cell current = maze(i, j);
					CharBlock cbCur = current.Serialize();
					if (j < 15)
					{
						Cell up = maze(i, j + 1);
						CharBlock cbUp = up.Serialize();
						WallState cu = current.GetUp();
						WallState ud = up.GetDown();
						if (cu != ud)
						{
							error = true;
						}
						if (current.GetUp() != up.GetDown())
						{
							std::wstringstream ss = std::wstringstream();
							/*ss << "Error: Expected " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(cu);
							ss << ", Found " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(ud);*/
							ss << "\nAt " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(current);
							ss << " and " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(up) << ".";
							Assert::AreEqual(current.GetUp(), up.GetDown(), (&ss.str()[0]));
						}
						
					}
					if (i < 15)
					{
						Cell right = maze(i+1, j);
						CharBlock cbRight = right.Serialize();

						WallState cr = current.GetRight();
						WallState rl = right.GetLeft();
						if (current.GetRight() != right.GetLeft())
						{
							error = true;
						}
						if (current.GetRight() != right.GetLeft())
						{
							std::wstringstream ss = std::wstringstream();
							/*ss << "Error: Expected " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(cr);
							ss << ", Found " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(rl);*/
							ss << "\nAt " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(current);
							ss << " and " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(right) << ".";
							Assert::AreEqual(current.GetRight(), right.GetLeft(), (&ss.str()[0]));
						}
					}

				}
			}
		}

		TEST_METHOD(TestUnitTestInfrastructure)
		{
			Maze m1 = Mazes::GetMaze1();
			Maze m2 = Mazes::GetMaze2();
			Maze m3 = Mazes::GetMaze2();

			ValidateMaze(m1);
			ValidateMaze(m2);
			ValidateMaze(m3);
		}
		TEST_METHOD(TestIsAccessibleLocation)
		{
			Maze m1 = Mazes::GetMaze1();

			for (uint8_t i = 0; i < 16; i++)
			{
				for (uint8_t j = 0; j < 16; j++)
				{
					for (Direction d = Direction::Up; d <= Direction::Right; d= d<<1)
					{
						MazeLocation l((i << 1) + 1, (j << 1) + 1);
						WallState s = m1(i, j).GetWall(d);
						l = l >> d;
						bool b2 = (s == WallState::NoWall) == m1.IsAccessibleLocation(l);
						bool b = m1.IsAccessibleLocation(l);
						Assert::AreEqual(s == WallState::NoWall, b);
						Assert::IsTrue(b2);
					}
				}
			}
		}
		TEST_METHOD(TestConstructor)
		{
			Maze m = Maze();
			Assert::AreEqual(0.18f, m.GetCellDimension(), 1.0e-6f);
			for (int i = 0; i < m.GetXSize(); i++)
			{
				for (int j = 0; j < m.GetYSize(); j++)
				{
					Cell& c = m(i, j);

					Assert::AreEqual((int)c.GetX(), i);
					Assert::AreEqual((int)c.GetY(), j);
					if (j == 15)
					{
						Assert::AreEqual(Wall, c.GetUp());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetUp());
					}
					if (j == 0)
					{
						Assert::AreEqual(Wall, c.GetDown());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetDown());
					}
					if (i == 0)
					{
						Assert::AreEqual(Wall, c.GetLeft());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetLeft());
					}
					if (i == 15)
					{
						Assert::AreEqual(Wall, c.GetRight());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetRight());
					}
				}
			}
		}
		TEST_METHOD(TestSize)
		{
			Maze m = Maze();
			int xSize = m.GetXSize();
			int ySize = m.GetYSize();
			Assert::AreEqual(m.GetXSize(), (uint8_t)16);
			Assert::AreEqual(m.GetYSize(), (uint8_t)16);
		}
		TEST_METHOD(TestIndex)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);

			c.SetUp(Wall);
			Assert::AreEqual(c.GetUp(), m(1, 1).GetUp());
		}
		TEST_METHOD(TestIndex2)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);

			m(1, 1).SetUp(Wall);
			Assert::AreEqual(c.GetUp(), m(1, 1).GetUp());
		}
		TEST_METHOD(TestIndex3)
		{
			Maze m = Maze();
			Cell& c = m(2, 2);

			m(2, 2).SetDown(NoWall);
			Assert::AreEqual(NoWall, m(2, 2).GetDown());
		}
		TEST_METHOD(TestIndex4)
		{
			Maze m = Maze();
			Cell& c = m(2, 2);

			c.SetDown(NoWall);
			Assert::AreEqual(NoWall, m(2, 2).GetDown());
		}
		TEST_METHOD(TestSetWallA)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Up, Wall);
			Assert::AreEqual(c.GetUp(), Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Wall);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallA2)
		{
			Maze m = Maze();
			Cell& c = m(1, 15);
			m.SetWall(c, Up, Wall);
			Assert::AreEqual(m(1, 15).GetUp(), Wall);
			Assert::AreEqual(m(1, 15).GetDown(), Unknown);
			Assert::AreEqual(m(1, 15).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 15).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallB)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Down, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Wall);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallB2)
		{
			Maze m = Maze();
			Cell& c = m(1, 0);
			m.SetWall(c, Down, Wall);
			Assert::AreEqual(m(1, 0).GetUp(), Unknown);
			Assert::AreEqual(m(1, 0).GetDown(), Wall);
			Assert::AreEqual(m(1, 0).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 0).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallC)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Left, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Wall);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallC2)
		{
			Maze m = Maze();
			Cell& c = m(0, 1);
			m.SetWall(c, Left, Wall);
			Assert::AreEqual(m(0, 1).GetUp(), Unknown);
			Assert::AreEqual(m(0, 1).GetDown(), Unknown);
			Assert::AreEqual(m(0, 1).GetLeft(), Wall);
			Assert::AreEqual(m(0, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallD)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Right, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Wall);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallD2)
		{
			Maze m = Maze();
			Cell& c = m(15, 1);
			m.SetWall(c, Right, Wall);
			Assert::AreEqual(m(15, 1).GetUp(), Unknown);
			Assert::AreEqual(m(15, 1).GetDown(), Unknown);
			Assert::AreEqual(m(15, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(15, 1).GetRight(), Wall);

			ValidateMaze(m);
		}
		TEST_METHOD(TestGoalFound)
		{
			Maze& m = Mazes::GetMaze1();

			Assert::IsTrue(m.HasFoundGoal());
		}
		TEST_METHOD(TestRefMazes)
		{
			Maze& m1 = Mazes::GetMaze1();

			Assert::IsTrue(m1(1, 2).GetUp() == WallState::NoWall);
			Assert::IsTrue(m1(1, 2).GetDown() == WallState::Wall);
			Assert::IsTrue(m1(1, 2).GetLeft() == WallState::Wall);
			Assert::IsTrue(m1(1, 2).GetRight() == WallState::Wall);

			Assert::IsTrue(m1(2, 1).GetUp() == WallState::NoWall);
			Assert::IsTrue(m1(2, 1).GetDown() == WallState::Wall);
			Assert::IsTrue(m1(2, 1).GetLeft() == WallState::NoWall);
			Assert::IsTrue(m1(2, 1).GetRight() == WallState::NoWall);


		}
		TEST_METHOD(TestRefMazes2)
		{
			Maze& m1 = Mazes::GetMaze1();
			ValidateMaze(m1);
		}

		TEST_METHOD(TestRefMazes3)
		{
			Maze& m2 = Mazes::GetMaze2();
			ValidateMaze(m2);
		}
		TEST_METHOD(TestIsComplete1)
		{
			Maze& m = Mazes::GetMaze1();

			Assert::IsTrue(m.IsComplete());
		}
		TEST_METHOD(TestIsCompleteSpeed1000)
		{
			Maze& m = Mazes::GetMaze1();
			m.IsComplete();
			WallState ws = m(0, 0).GetUp();
			auto start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < 100; ++i)
			{
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();

				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
			}
			auto end = std::chrono::high_resolution_clock::now();
			auto duration = end - start;
			Assert::IsTrue(m.IsComplete());
		}
		TEST_METHOD(TestIsComplete1b)
		{
			Maze m = Mazes::GetMaze1();
			m.SetWall(m(3, 4), Direction::Up, WallState::Unknown);

			Assert::AreEqual(WallState::Unknown, m(3, 4).GetUp());
			Assert::IsFalse(m(3, 4).IsFullyKnown());

			Assert::IsFalse(m.IsComplete());
		}
		TEST_METHOD(TestIsComplete1c)
		{
			Maze m = Mazes::GetMaze1();
			Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::IsTrue(P.GetSize() > 40);
			return;
		}
		TEST_METHOD(TestPathFinding)
		{
			Maze m = Mazes::GetMaze1();
			Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(CellCoordinates(0, 0), P.first());

			int dx = P.last().GetX() - m.GetGoalLowerLeft().GetX();
			int dy = P.last().GetY() - m.GetGoalLowerLeft().GetY();

			Assert::IsTrue(dx >= 0);
			Assert::IsTrue(dx <= 1);
			Assert::IsTrue(dy >= 0);
			Assert::IsTrue(dy <= 1);
			return;
		}
		TEST_METHOD(TestPathFindingContinuity)
		{
			Maze m = Mazes::GetMaze1();
			Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(CellCoordinates(0, 0), P.first());

			for (uint16_t i = 1; i < P.GetSize(); i++)
			{
				Direction prevDirection = P[i].DirectionTo(P[i - 1]);
				Assert::AreEqual(P[i - 1], P[i] >> prevDirection);
			}

			return;
		}
		TEST_METHOD(TestIsComplete2)
		{
			Maze& m = Mazes::GetMaze2();
			bool result = m.IsComplete();
			Assert::IsTrue(result);
		}
		TEST_METHOD(TestGoal)
		{
			Maze& m = Mazes::GetSingleTurnMaze();
			bool result = m.IsComplete();
			Assert::IsTrue(result);

			CellCoordinates coords = m.GetGoalLowerLeft();
			Assert::AreEqual(1, (int)coords.GetX());
			Assert::AreEqual(14, (int)coords.GetY());
		}

		TEST_METHOD(FillMazeTest)
		{
			Maze& m = Mazes::GetMaze4();
			Maze simulated = Maze();
			Vehicle testVehicle;
			FloodFillPathFinder ffpf = FloodFillPathFinder(simulated, testVehicle);

			CellCoordinates current = CellCoordinates(0, 0);
			for (Direction dir = Direction::Up; dir <= Direction::Right; dir = dir << 1)
			{
				simulated.SetWall(simulated[current], dir, m[current].GetWall(dir));
			}
			Direction cDir = Direction::Up;
			while (!simulated.IsComplete())
			{
				HalfStepPath<PATH_SIZE*2> p;
				ffpf.HalfStepPathToNearestUnknown(current, cDir, p);

				current = static_cast<CellCoordinates>(p.last());
				cDir = p.last(1).DirectionTo(p.last());
				for (Direction dir = Direction::Up; dir <= Direction::Right; dir = dir << 1)
				{
					simulated.SetWall(simulated[current], dir, m[current].GetWall(dir));
				}
			}
			Assert::IsTrue(simulated.IsComplete());
		}
	};
}

