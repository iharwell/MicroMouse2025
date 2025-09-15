#pragma once

// add headers that you want to pre-compile here

#include "..\MazeMap\Cell.h"
#include "..\MazeMap\Maze.h"
#include "CppUnitTest.h"
#include <sstream>


template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(MazeMap::WallState const& state)
{
	switch (state)
	{
	case MazeMap::WallState::Unknown:
		return std::wstring(L"U");
	case MazeMap::WallState::NoWall:
		return std::wstring(L"N");
	case MazeMap::WallState::Wall:
	default:
		return std::wstring(L"W");
	}
}
template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(MazeMap::Cell const& cell)
{
	std::wstringstream s = std::wstringstream();
	s << "(" << cell.GetX() << ", " << cell.GetY() << "): ";
	s << ToString(cell.GetUp()) << ToString(cell.GetDown()) << ToString(cell.GetLeft()) << ToString(cell.GetRight());
	return std::wstring(s.str());
}
template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(MazeMap::Maze const& state)
{
	return std::wstring();
}