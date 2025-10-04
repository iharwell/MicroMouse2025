#pragma once

#include "Defines.h"

namespace MazeMap
{
	template <typename T>
	class EXPORT TimedValue
	{
	private:
		time_t _time;
		T _value;
	public:
		TimedValue(T value = T())
			: _value(value)
			, _time(GetPreciseTime())
		{}
		TimedValue(T value, time_t time)
			: _value(value)
			, _time(time)
		{}

		time_t GetTicks() const { return _time; }
		time_t GetTicks()       { return _time; }

		const T& GetValue() const { return _value; }
		      T& GetValue()       { return _value; }

		time_t GetAge() const { return GetPreciseTime() - _time; }
		time_t GetAge()       { return GetPreciseTime() - _time; }
	};

	/*template<typename T, int _SIZE>
	class EXPORT TimedCircularBuffer
	{
	private:
		TimedValue<T> _data[_SIZE];
		int _first;
		int _count;
	public:
		TimedCircularBuffer()
			: _first(0)
			, _count(0)
		{}

		bool push_back(const T& value)
		{
			if (_count < _SIZE)
			{
				_data[_count] = value;
				++_count;
				return true;
			}
			else
			{
				_data[_first] = value;
				++_first;
				if (_first >= _SIZE)
				{
					_first -= _SIZE;
				}
				return true;
			}
		}

		const T& index(int i) const
		{
			int index = (i + _first) % _SIZE;
			return _data[index];
		}

		const T& index(int i) { return const_cast<const TimedCircularBuffer*>(this)->index(i); }

		int size() const { return _count; }
		int size() { return _count; }

		int maxSize() const { return _SIZE; }
		int maxSize() { return _SIZE; }
	};*/

	template<typename T, int _SIZE>
	class EXPORT CircularBuffer
	{
	private:
		T _data[_SIZE];
		int _first;
		int _count;
	public:
		CircularBuffer()
			: _first(0)
			, _count(0)
		{}

		bool push_back(const T& value)
		{
			if (_count < _SIZE)
			{
				_data[_count] = value;
				++_count;
				return true;
			}
			else
			{
				_data[_first] = value;
				++_first;
				if (_first >= _SIZE)
				{
					_first -= _SIZE;
				}
				return true;
			}
		}

		const T& operator[] (int i) const { return index(i); }
		const T& operator[] (int i) { return const_cast<const CircularBuffer*>(this)->index(i); }

		const T& index(int i) const
		{
			int indNum = (i + _first) % _SIZE;
			return _data[indNum];
		}
		const T& index(int i) { return const_cast<const CircularBuffer*>(this)->index(i); }

		const T& GetLatest() const { return (*this)[_count - 1]; }
		const T& GetLatest() { return const_cast<const CircularBuffer*>(this)->GetLatest(); }

		int size() const { return _count; }
		int size() { return _count; }

		int maxSize() const { return _SIZE; }
		int maxSize() { return _SIZE; }
	};
}
