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
		return std::wstring(L"O");
	case MazeMap::WallState::Wall:
	default:
		return std::wstring(L"W");
	}
}
template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(MazeMap::Direction const& direction)
{
	switch (direction)
	{
	case MazeMap::Direction::Up:
		return std::wstring(L"UU");
	case MazeMap::Direction::UpRight:
		return std::wstring(L"UR");
	case MazeMap::Direction::Right:
		return std::wstring(L"RR");
	case MazeMap::Direction::DownRight:
		return std::wstring(L"DR");

	case MazeMap::Direction::Down:
		return std::wstring(L"DD");
	case MazeMap::Direction::DownLeft:
		return std::wstring(L"DL");
	case MazeMap::Direction::Left:
		return std::wstring(L"LL");
	case MazeMap::Direction::UpLeft:
		return std::wstring(L"UL");
	default:
		return std::wstring(L"W");
	}
}
template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(MazeMap::RelativeDirection const& direction)
{
	switch (direction)
	{
	case MazeMap::RelativeDirection::F:
		return std::wstring(L"F");
	case MazeMap::RelativeDirection::Reverse:
		return std::wstring(L"B");

	case MazeMap::RelativeDirection::R45:
		return std::wstring(L"R45");
	case MazeMap::RelativeDirection::R90:
		return std::wstring(L"R90");
	case MazeMap::RelativeDirection::R135:
		return std::wstring(L"R135");

	case MazeMap::RelativeDirection::L45:
		return std::wstring(L"L45");
	case MazeMap::RelativeDirection::L90:
		return std::wstring(L"L90");
	case MazeMap::RelativeDirection::L135:
		return std::wstring(L"L135");

	default:
		return std::wstring(L"W");
	}
}
template<>
inline std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString(const MazeMap::CellCoordinates& coords)
{
	std::wstringstream s = std::wstringstream();
	s << "(" << static_cast<int>(coords.GetX()) << ", " << static_cast<int>(coords.GetY()) << ")";

	return std::wstring(s.str());
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