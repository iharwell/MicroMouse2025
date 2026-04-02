#ifndef RUNTIMETEXTBLOCKBUILDER_H
#define RUNTIMETEXTBLOCKBUILDER_H

// Declares the metadata and notes text builder used by runtime MMLOG writers.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdio.h>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Accumulates line-oriented metadata blocks for MMLOG files and their sidecar descriptions.
				template <std::size_t TCapacityBytes = 8192U>
				class RuntimeTextBlockBuilder
				{
				private:
					static constexpr std::size_t kFormatBufferBytes = 192U;
					std::array<char, TCapacityBytes + 1U> _buffer;
					std::size_t _length;

					bool AppendFormattedLine(const char* line, int length);

				public:
					RuntimeTextBlockBuilder();

					void Clear();
					bool AppendLine(const char* line);
					bool AppendKeyValue(const char* key, const char* value);
					bool AppendUnsigned(const char* key, unsigned long value);
					bool AppendFloat(const char* key, float value, uint8_t precision);

					// Returns the accumulated block contents or null when no lines have been appended.
					const char* Data();
					// Returns the accumulated block contents or null when no lines have been appended.
					const char* Data() const;
				};

				template <std::size_t TCapacityBytes>
				RuntimeTextBlockBuilder<TCapacityBytes>::RuntimeTextBlockBuilder()
					: _buffer{}
					, _length(0U)
				{
					_buffer[0] = '\0';
				}

				template <std::size_t TCapacityBytes>
				void RuntimeTextBlockBuilder<TCapacityBytes>::Clear()
				{
					_length = 0U;
					_buffer[0] = '\0';
				}

				template <std::size_t TCapacityBytes>
				bool RuntimeTextBlockBuilder<TCapacityBytes>::AppendLine(const char* line)
				{
					if (line == nullptr)
					{
						return false;
					}

					const std::size_t lineLength = std::strlen(line);
					if ((_length + lineLength + 1U) >= _buffer.size())
					{
						return false;
					}

					std::memcpy(_buffer.data() + _length, line, lineLength);
					_length += lineLength;
					_buffer[_length++] = '\n';
					_buffer[_length] = '\0';
					return true;
				}

				template <std::size_t TCapacityBytes>
				bool RuntimeTextBlockBuilder<TCapacityBytes>::AppendKeyValue(const char* key, const char* value)
				{
					char line[kFormatBufferBytes] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s=%s",
						(key != nullptr) ? key : "",
						(value != nullptr) ? value : "");
					return AppendFormattedLine(line, length);
				}

				template <std::size_t TCapacityBytes>
				bool RuntimeTextBlockBuilder<TCapacityBytes>::AppendUnsigned(const char* key, unsigned long value)
				{
					char line[kFormatBufferBytes] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s=%lu",
						(key != nullptr) ? key : "",
						value);
					return AppendFormattedLine(line, length);
				}

				template <std::size_t TCapacityBytes>
				bool RuntimeTextBlockBuilder<TCapacityBytes>::AppendFloat(const char* key, float value, uint8_t precision)
				{
					char line[kFormatBufferBytes] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s=%.*f",
						(key != nullptr) ? key : "",
						static_cast<int>(precision),
						value);
					return AppendFormattedLine(line, length);
				}

				template <std::size_t TCapacityBytes>
				const char* RuntimeTextBlockBuilder<TCapacityBytes>::Data() const
				{
					return (_length > 0U) ? _buffer.data() : nullptr;
				}

				template <std::size_t TCapacityBytes>
				const char* RuntimeTextBlockBuilder<TCapacityBytes>::Data()
				{
					return const_cast<RuntimeTextBlockBuilder const*>(this)->Data();
				}

				template <std::size_t TCapacityBytes>
				bool RuntimeTextBlockBuilder<TCapacityBytes>::AppendFormattedLine(const char* line, int length)
				{
					if (length <= 0 || length >= static_cast<int>(kFormatBufferBytes))
					{
						return false;
					}

					return AppendLine(line);
				}
			}
		}
	}
}

#endif
