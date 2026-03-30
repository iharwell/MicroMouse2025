#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverQueue.h"
#pragma once

#include "Defines.h"
#include "ManeuverInstance.h"
#include "ManeuverPath.h"
#include "Maze.h"
#include "Vehicle.h"

namespace MazeMap
{
	constexpr uint16_t MANEUVER_QUEUE_CAPACITY = PATH_SIZE*2;

	class ManeuverQueue
	{
	private:
		ManeuverInstance _entries[MANEUVER_QUEUE_CAPACITY];
		uint16_t _size;

		static MAZEMAP_INLINE bool IsStraightCode(ManeuverCode code)
		{
			return code != MC_NONE && code <= S31;
		}

		static MAZEMAP_INLINE float ClampNonNegative(float value)
		{
			return (value < 0.0f) ? 0.0f : value;
		}

		static MAZEMAP_INLINE float GetStraightDistanceMeters(ManeuverCode code)
		{
			return static_cast<float>(static_cast<uint8_t>(code)) * 0.5f * Maze::GetCellDimension() / 100.0f;
		}

		static MAZEMAP_INLINE float GetSpeedLimit(ManeuverCode code, const Vehicle& vehicle)
		{
			if (code == MC_NONE)
			{
				return 0.0f;
			}
			if (IsStraightCode(code))
			{
				return vehicle.GetMaxSpeed();
			}
			return ManeuverSet::GetSet()[code].GetVMax(vehicle);
		}

		static MAZEMAP_INLINE float ReachableSpeed(float boundarySpeed, float distance, const Vehicle& vehicle)
		{
			float speedSq = (boundarySpeed * boundarySpeed) + (2.0f * vehicle.GetMaxForwardAcceleration() * distance);
			return (speedSq <= 0.0f) ? 0.0f : sqrtf(speedSq);
		}
	public:
		MAZEMAP_INLINE ManeuverQueue()
			: _entries()
			, _size(0)
		{
		}

		MAZEMAP_INLINE ManeuverQueue(const ManeuverPath& path, DirectionalLocation start)
			: _entries()
			, _size(0)
		{
			push_back(path, start);
		}

		MAZEMAP_INLINE uint16_t size() const { return _size; }
		MAZEMAP_INLINE uint16_t size() { return const_cast<const ManeuverQueue*>(this)->size(); }
		MAZEMAP_INLINE uint16_t maxSize() const { return MANEUVER_QUEUE_CAPACITY; }
		MAZEMAP_INLINE uint16_t maxSize() { return const_cast<const ManeuverQueue*>(this)->maxSize(); }
		MAZEMAP_INLINE bool empty() const { return _size == 0; }
		MAZEMAP_INLINE bool empty() { return const_cast<const ManeuverQueue*>(this)->empty(); }
		MAZEMAP_INLINE bool full() const { return _size >= MANEUVER_QUEUE_CAPACITY; }
		MAZEMAP_INLINE bool full() { return const_cast<const ManeuverQueue*>(this)->full(); }

		MAZEMAP_INLINE void clear()
		{
			_size = 0;
		}

		MAZEMAP_INLINE ManeuverInstance& operator[](uint16_t index) { return _entries[index]; }
		MAZEMAP_INLINE const ManeuverInstance& operator[](uint16_t index) const { return _entries[index]; }

		MAZEMAP_INLINE ManeuverInstance& front() { return _entries[0]; }
		MAZEMAP_INLINE const ManeuverInstance& front() const { return _entries[0]; }
		MAZEMAP_INLINE ManeuverInstance& back() { return _entries[_size - 1]; }
		MAZEMAP_INLINE const ManeuverInstance& back() const { return _entries[_size - 1]; }

		MAZEMAP_INLINE DirectionalLocation GetNextStart() const
		{
			return empty() ? DirectionalLocation() : back().GetEnd();
		}

		MAZEMAP_INLINE bool push_back(const ManeuverInstance& instance)
		{
			if (full() || instance.GetCode() == MC_NONE)
			{
				return false;
			}

			if (!empty() && !(back().GetEnd() == instance.GetStart()))
			{
				return false;
			}

			_entries[_size] = instance;
			++_size;
			return true;
		}

		MAZEMAP_INLINE bool push_back(ManeuverCode code, DirectionalLocation start)
		{
			return push_back(ManeuverInstance(code, start));
		}

		MAZEMAP_INLINE bool push_back(ManeuverCode code)
		{
			if (empty())
			{
				return false;
			}

			return push_back(code, back().GetEnd());
		}

		MAZEMAP_INLINE bool push_back(const ManeuverPath& path, DirectionalLocation start)
		{
			if ((_size + path.GetSize()) > MANEUVER_QUEUE_CAPACITY)
			{
				return false;
			}

			if (!empty() && !(back().GetEnd() == start))
			{
				return false;
			}

			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				if (path[i] == MC_NONE)
				{
					return false;
				}
			}

			DirectionalLocation current = start;
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				_entries[_size] = ManeuverInstance(path[i], current);
				current = _entries[_size].GetEnd();
				++_size;
			}
			return true;
		}

		MAZEMAP_INLINE bool push_back(const ManeuverPath& path)
		{
			if (path.GetSize() == 0)
			{
				return true;
			}

			if (empty())
			{
				return false;
			}

			return push_back(path, back().GetEnd());
		}

		MAZEMAP_INLINE bool pop_front()
		{
			if (empty())
			{
				return false;
			}

			for (uint16_t i = 1; i < _size; ++i)
			{
				_entries[i - 1] = _entries[i];
			}
			--_size;
			return true;
		}

		MAZEMAP_INLINE bool pop_front(ManeuverInstance& removed)
		{
			if (empty())
			{
				return false;
			}

			removed = front();
			return pop_front();
		}

		static MAZEMAP_INLINE void ComputeSpeeds(
			const Vehicle& vehicle,
			ManeuverInstance* entries,
			uint16_t count,
			float initialEntrySpeed = 0.0f,
			float finalExitSpeed = -1.0f)
		{
			if (entries == nullptr || count == 0)
			{
				return;
			}

			float currentBoundarySpeed = ClampNonNegative(initialEntrySpeed);
			for (uint16_t i = 0; i < count; ++i)
			{
				ManeuverInstance& entry = entries[i];
				float speedLimit = GetSpeedLimit(entry.GetCode(), vehicle);

				if (IsStraightCode(entry.GetCode()))
				{
					float entrySpeed = std::min(currentBoundarySpeed, speedLimit);
					float exitSpeed = std::min(speedLimit, ReachableSpeed(entrySpeed, GetStraightDistanceMeters(entry.GetCode()), vehicle));
					entry.SetEntrySpeed(entrySpeed);
					entry.SetExitSpeed(exitSpeed);
					currentBoundarySpeed = exitSpeed;
				}
				else
				{
					float maneuverSpeed = std::min(currentBoundarySpeed, speedLimit);
					entry.SetEntrySpeed(maneuverSpeed);
					entry.SetExitSpeed(maneuverSpeed);
					currentBoundarySpeed = maneuverSpeed;
				}
			}

			float requiredExitSpeed = (finalExitSpeed < 0.0f) ? vehicle.GetMaxSpeed() : ClampNonNegative(finalExitSpeed);
			for (int i = static_cast<int>(count) - 1; i >= 0; --i)
			{
				ManeuverInstance& entry = entries[i];
				float speedLimit = GetSpeedLimit(entry.GetCode(), vehicle);

				if (IsStraightCode(entry.GetCode()))
				{
					float exitSpeed = std::min(entry.GetExitSpeed(), std::min(requiredExitSpeed, speedLimit));
					float entrySpeed = std::min(entry.GetEntrySpeed(), std::min(speedLimit, ReachableSpeed(exitSpeed, GetStraightDistanceMeters(entry.GetCode()), vehicle)));
					entry.SetEntrySpeed(entrySpeed);
					entry.SetExitSpeed(exitSpeed);
					requiredExitSpeed = entrySpeed;
				}
				else
				{
					float maneuverSpeed = std::min(entry.GetEntrySpeed(), std::min(requiredExitSpeed, speedLimit));
					entry.SetEntrySpeed(maneuverSpeed);
					entry.SetExitSpeed(maneuverSpeed);
					requiredExitSpeed = maneuverSpeed;
				}
			}
		}

		MAZEMAP_INLINE void ComputeSpeeds(
			const Vehicle& vehicle,
			float initialEntrySpeed = 0.0f,
			float finalExitSpeed = -1.0f)
		{
			ComputeSpeeds(vehicle, _entries, _size, initialEntrySpeed, finalExitSpeed);
		}

		// Fresh additions can tighten speeds on earlier straights, so this recomputes the full queue.
		MAZEMAP_INLINE void ComputeFreshSpeeds(
			const Vehicle& vehicle,
			uint16_t freshEntryCount,
			float initialEntrySpeed = 0.0f,
			float finalExitSpeed = -1.0f)
		{
			if (freshEntryCount == 0 || empty())
			{
				return;
			}

			ComputeSpeeds(vehicle, initialEntrySpeed, finalExitSpeed);
		}
	};
}


