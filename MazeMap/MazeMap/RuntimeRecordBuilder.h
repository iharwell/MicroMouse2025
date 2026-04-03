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
				template <std::size_t SIZE>
				class RuntimeRecordBuilder
				{
				private:
					std::array<uint8_t, SIZE> _words;
					std::size_t _index;
					bool HasRoom(uint8_t val);
					bool HasRoom(uint16_t val);
					bool HasRoom(uint32_t val);
					bool HasRoom(int8_t val);
					bool HasRoom(int16_t val);
					bool HasRoom(int32_t val);
					bool HasRoom(float val);
				public:
					RuntimeRecordBuilder();

					void Clear();
					void U8(uint8_t value);
					void U16(uint16_t value);
					void U32(uint32_t value);
					void I8(int8_t value);
					void I16(int16_t value);
					void I32(int32_t value);
					void F32(float value);

					// Returns the packed row words in MMLOG order.
					const uint8_t* Data();
					// Returns the packed row words in MMLOG order.
					const uint8_t* Data() const;

					uint32_t Count();
					uint32_t Count() const;
					bool IsFull();
					bool IsFull() const;
				};

				template <std::size_t SIZE>
				RuntimeRecordBuilder<SIZE>::RuntimeRecordBuilder()
					: _words{}
					, _index(0U)
				{
					_words.fill(0U);
				}

				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::Clear()
				{
					_words.fill(0U);
					_index = 0U;
				}
				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::U8(uint8_t value)
				{
					if (HasRoom(value))
					{
						_words[_index++] = (value);
					}
				}
				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::U16(uint16_t value)
				{
					if (HasRoom(value))
					{
						uint16_t* current = reinterpret_cast<uint16_t*>(&_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}

				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::U32(uint32_t value)
				{
					if (HasRoom(value))
					{
						uint32_t* current = reinterpret_cast<uint32_t*>( &_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}
				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::I8(int8_t value)
				{
					if (HasRoom(value))
					{
						int8_t* current = reinterpret_cast<int8_t*>(&_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}
				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::I16(int16_t value)
				{
					if (HasRoom(value))
					{
						int16_t* current = reinterpret_cast<int16_t*>(&_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}
				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::I32(int32_t value)
				{
					if (HasRoom(value))
					{
						int32_t* current = reinterpret_cast<int32_t*>(&_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}

				template <std::size_t SIZE>
				void RuntimeRecordBuilder<SIZE>::F32(float value)
				{
					if (HasRoom(value))
					{
						float* current = reinterpret_cast<float*>(&_words[_index]);
						*current = (value);
						_index += sizeof(value);
					}
				}

				template <std::size_t SIZE>
				const uint8_t* RuntimeRecordBuilder<SIZE>::Data() const
				{
					return _words.data();
				}

				template <std::size_t SIZE>
				const uint8_t* RuntimeRecordBuilder<SIZE>::Data()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->Data();
				}

				template <std::size_t SIZE>
				uint32_t RuntimeRecordBuilder<SIZE>::Count() const
				{
					return static_cast<uint32_t>(SIZE);
				}

				template <std::size_t SIZE>
				uint32_t RuntimeRecordBuilder<SIZE>::Count()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->Count();
				}

				template <std::size_t SIZE>
				bool RuntimeRecordBuilder<SIZE>::IsFull() const
				{
					return _index == SIZE;
				}

				template <std::size_t SIZE>
				bool RuntimeRecordBuilder<SIZE>::IsFull()
				{
					return const_cast<RuntimeRecordBuilder const*>(this)->IsFull();
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(uint8_t val)
				{
					return _index + 1 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(uint16_t val)
				{
					return _index + 2 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(uint32_t val)
				{
					return _index + 4 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(int8_t val)
				{
					return _index + 1 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(int16_t val)
				{
					return _index + 2 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(int32_t val)
				{
					return _index + 4 < SIZE;
				}
				template<std::size_t SIZE>
				inline bool RuntimeRecordBuilder<SIZE>::HasRoom(float val)
				{
					return _index + 4 < SIZE;
				}
}
		}
	}
}

#endif
