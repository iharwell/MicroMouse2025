#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\DirectionalLocation.cpp"
#include "pch.h"
#include "DirectionalLocation.h"

namespace MazeMap
{
	DirectionalLocation::DirectionalLocation()
		: _loc()
		, _dir(Direction::Up)
	{ }
	DirectionalLocation::DirectionalLocation(MazeLocation loc, Direction dir)
		: _loc(loc)
		, _dir(dir)
	{ }
	DirectionalLocation::DirectionalLocation(uint8_t halfX, uint8_t halfY, Direction dir)
		: _loc(halfX, halfY)
		, _dir(dir)
	{
	}
	MazeLocation DirectionalLocation::GetLocation() { return const_cast<const DirectionalLocation*>(this)->GetLocation(); }
	MazeLocation DirectionalLocation::GetLocation() const {return _loc;}
	Direction DirectionalLocation::GetDirection() { return const_cast<const DirectionalLocation*>(this)->GetDirection(); }
	Direction DirectionalLocation::GetDirection() const { return _dir; }

	CellCoordinates DirectionalLocation::GetFollowingCell()
	{
		bool isHWallLoc = (_loc.GetY() & 1) == 0;
		bool isVWallLoc = (_loc.GetX() & 1) == 0;
		if (isHWallLoc)
		{
			if ((_dir & Direction::Up) == Direction::Up)
			{
				return CellCoordinates(_loc.GetX() >> 1, _loc.GetY() >> 1);
			}
			else
			{
				return CellCoordinates(_loc.GetX() >> 1, (_loc.GetY() - 1) >> 1);
			}
		}
		if (isVWallLoc)
		{
			if ((_dir & Direction::Right) == Direction::Right)
			{
				return CellCoordinates(_loc.GetX() >> 1, _loc.GetY() >> 1);
			}
			else
			{
				return CellCoordinates((_loc.GetX()-1) >> 1, _loc.GetY() >> 1);
			}
		}
		return static_cast<CellCoordinates>(_loc);
	}

	DirectionalLocation DirectionalLocation::Turn(RelativeDirection relDir)
	{
		return DirectionalLocation(GetLocation(), GetDirection() + relDir);
	}
	/*DirectionalLocation DirectionalLocation::MoveForward(uint8_t halfSteps)
	{
		int8_t dx = 0;
		int8_t dy = 0;
		if ((GetDirection() & Direction::Up))
		{
			dy = halfSteps;
		}
		else if ((GetDirection() & Direction::Down))
		{
			dy = -halfSteps;
		}
		if ((GetDirection() & Direction::Left))
		{
			dx = -halfSteps;
		}
		else if ((GetDirection() & Direction::Right))
		{
			dx = halfSteps;
		}
		return DirectionalLocation(_loc.GetX() + dx, _loc.GetY() + dy, _dir);
	}*/
	DirectionalLocation DirectionalLocation::operator>>(uint8_t halfSteps)
	{
		return MoveForward(halfSteps);
	}
	DirectionalLocation DirectionalLocation::operator>>(RelativeDirection relDir)
	{
		return Turn(relDir);
	}
	/*DirectionalLocation DirectionalLocation::operator>>(RelativeDirectionalDistance instruction)
	{
		DirectionalLocation loc = (*this);
		loc = loc.Turn(instruction.GetDirection());
		loc = loc.MoveForward(instruction.GetDistance());
		return loc;
	}*/
	bool DirectionalLocation::operator==(const DirectionalLocation& other)
	{
		return (other.GetDirection() == GetDirection()) && (other.GetLocation() == GetLocation());
	}
	bool DirectionalLocation::operator==(const DirectionalLocation& other) const
	{
		return (other.GetDirection() == GetDirection()) && (other.GetLocation() == GetLocation());
	}
}