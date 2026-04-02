#ifndef RUNTIMERECORDBUILDER_H
#define RUNTIMERECORDBUILDER_H

// Declares the typed record builder used to pack runtime MMLOG rows.

#include "MmLog.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Packs a fixed-width MMLOG row one field at a time before the row is appended to disk.
				template <std::size_t TFieldCount>
				class RuntimeRecordBuilder
				{
				private:
					std::array<uint32_t, TFieldCount> _words;
					std::size_t _index;

				public:
					RuntimeRecordBuilder();

					void Clear();
					void U32(uint32_t value);
					void I32(int32_t value);
					void F32(float value);

					// Returns the packed row words in MMLOG order.
					const uint32_t* Data();
					// Returns the packed row words in MMLOG order.
					const uint32_t* Data() const;

					uint32_t Count();
					uint32_t Count() const;
					bool IsFull();
					bool IsFull() const;
				};

				template <std::size_t TFieldCount>
				RuntimeRecordBuilder<TFieldCount>::RuntimeRecordBuilder()
					: _words{}
					, _index(0U)
				{
					_words.fill(0U);
				}

				template <std::size_t TFieldCount>
				void RuntimeRecordBuilder<TFieldCount>::Clear()
				{
					_words.fill(0U);
					_index = 0U;
				}

				template <std::size_t TFieldCount>
				void RuntimeRecordBuilder<TFieldCount>::U32(uint32_t value)
				{
					if (_index < TFieldCount)
					{
						_words[_index++] = mmlog::packU32(value);
					}
				}

				template <std::size_t TFieldCount>
				void RuntimeRecordBuilder<TFieldCount>::I32(int32_t value)
				{
					if (_index < TFieldCount)
					{
						_words[_index++] = mmlog::packI32(value);
					}
				}

				template <std::size_t TFieldCount>
				void RuntimeRecordBuilder<TFieldCount>::F32(float value)
				{
					if (_index < TFieldCount)
					{
						_words[_index++] = mmlog::packF32(value);
					}
				}

				template <std::size_t TFieldCount>
				const uint32_t* RuntimeRecordBuilder<TFieldCount>::Data() const
				{
					return _words.data();
				}

				template <std::size_t TFieldCount>
				const uint32_t* RuntimeRecordBuilder<TFieldCount>::Data()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->Data();
				}

				template <std::size_t TFieldCount>
				uint32_t RuntimeRecordBuilder<TFieldCount>::Count() const
				{
					return static_cast<uint32_t>(TFieldCount);
				}

				template <std::size_t TFieldCount>
				uint32_t RuntimeRecordBuilder<TFieldCount>::Count()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->Count();
				}

				template <std::size_t TFieldCount>
				bool RuntimeRecordBuilder<TFieldCount>::IsFull() const
				{
					return _index == TFieldCount;
				}

				template <std::size_t TFieldCount>
				bool RuntimeRecordBuilder<TFieldCount>::IsFull()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->IsFull();
				}
			}
		}
	}
}

#endif
