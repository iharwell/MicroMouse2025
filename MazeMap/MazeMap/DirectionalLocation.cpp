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
	}

	DirectionalLocation DirectionalLocation::Turn(RelativeDirection relDir)
	{
		return DirectionalLocation(GetLocation(), GetDirection() + relDir);
	}
	DirectionalLocation DirectionalLocation::MoveForward(uint8_t halfSteps)
	{
		int8_t dx = 0;
		int8_t dy = 0;
		if ((GetDirection() & Direction::Up) == Direction::Up)
		{
			dy = halfSteps;
		}
		else if ((GetDirection() & Direction::Down) == Direction::Down)
		{
			dy = -halfSteps;
		}
		if ((GetDirection() & Direction::Left) == Direction::Left)
		{
			dx = -halfSteps;
		}
		else if ((GetDirection() & Direction::Right) == Direction::Right)
		{
			dx = halfSteps;
		}
		MazeLocation loc = GetLocation();
		loc = MazeLocation(loc.GetX() + dx, loc.GetY() + dy);
		return DirectionalLocation(loc, GetDirection());
	}
	DirectionalLocation DirectionalLocation::operator>>(uint8_t halfSteps)
	{
		return MoveForward(halfSteps);
	}
	DirectionalLocation DirectionalLocation::operator>>(RelativeDirection relDir)
	{
		return Turn(relDir);
	}
}