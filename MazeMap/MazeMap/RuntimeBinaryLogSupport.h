#ifndef RUNTIMEBINARYLOGSUPPORT_H
#define RUNTIMEBINARYLOGSUPPORT_H

// Defines shared MMLOG support utilities for runtime file naming, schema validation, and label packing.

#include "MmLog.h"

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#endif

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
				enum class RuntimeBinaryLogWriteMode : uint8_t
				{
					Synchronous = 0u,
					Buffered = 1u
				};

				// Keeps runtime logging synchronous by default unless a caller explicitly opts into buffering.
				inline constexpr RuntimeBinaryLogWriteMode kDefaultRuntimeBinaryLogWriteMode =
					RuntimeBinaryLogWriteMode::Synchronous;

				inline bool SelectSequentialRuntimeFileName(
					char* buffer,
					std::size_t bufferSize,
					const char* explicitFileName,
					const char* teensyFormat,
					const char* hostFallback)
				{
					if (buffer == nullptr || bufferSize == 0U)
					{
						return false;
					}

					if (explicitFileName != nullptr && explicitFileName[0] != '\0')
					{
						snprintf(buffer, bufferSize, "%s", explicitFileName);
						return true;
					}

#if defined(ARDUINO_TEENSY41)
					if (teensyFormat == nullptr || teensyFormat[0] == '\0')
					{
						return false;
					}

					for (uint16_t index = 0U; index < 1000U; ++index)
					{
						snprintf(buffer, bufferSize, teensyFormat, static_cast<unsigned>(index));
						if (!SD.exists(buffer))
						{
							return true;
						}
					}

					snprintf(buffer, bufferSize, teensyFormat, 999U);
					return true;
#else
					(void)teensyFormat;
					snprintf(buffer, bufferSize, "%s", (hostFallback != nullptr) ? hostFallback : "runtime_log.mmlog");
					return true;
#endif
				}

				inline bool BuildSiblingRuntimeFileName(
					char* buffer,
					std::size_t bufferSize,
					const char* fileName,
					const char* siblingSuffix)
				{
					if (buffer == nullptr ||
						bufferSize == 0U ||
						fileName == nullptr ||
						fileName[0] == '\0' ||
						siblingSuffix == nullptr ||
						siblingSuffix[0] == '\0')
					{
						return false;
					}

					const char* extension = std::strrchr(fileName, '.');
					const std::size_t baseLength =
						(extension != nullptr) ? static_cast<std::size_t>(extension - fileName) : std::strlen(fileName);
					const std::size_t suffixLength = std::strlen(siblingSuffix);
					if ((baseLength + suffixLength + 1U) > bufferSize)
					{
						return false;
					}

					std::memcpy(buffer, fileName, baseLength);
					std::memcpy(buffer + baseLength, siblingSuffix, suffixLength);
					buffer[baseLength + suffixLength] = '\0';
					return true;
				}

				inline const char* FileNameComponent(const char* path) noexcept
				{
					if (path == nullptr)
					{
						return "";
					}

					const char* name = path;
					for (const char* cursor = path; *cursor != '\0'; ++cursor)
					{
						if (*cursor == '/' || *cursor == '\\')
						{
							name = cursor + 1;
						}
					}
					return name;
				}

				inline uint32_t Fnv1a32(const char* text) noexcept
				{
					uint32_t hash = 2166136261u;
					if (text == nullptr)
					{
						return hash;
					}

					for (const uint8_t* cursor = reinterpret_cast<const uint8_t*>(text); *cursor != 0u; ++cursor)
					{
						hash ^= *cursor;
						hash *= 16777619u;
					}
					return hash;
				}

				inline bool IsSchemaPrefix(const char* prefix, std::size_t length) noexcept
				{
					switch (length)
					{
					case 2U:
						return
							(std::strncmp(prefix, "u8", 2U) == 0) ||
							(std::strncmp(prefix, "i8", 2U) == 0) ||
							(std::strncmp(prefix, "s8", 2U) == 0);
					case 3U:
						return
							(std::strncmp(prefix, "u16", 3U) == 0) ||
							(std::strncmp(prefix, "i16", 3U) == 0) ||
							(std::strncmp(prefix, "u32", 3U) == 0) ||
							(std::strncmp(prefix, "i32", 3U) == 0) ||
							(std::strncmp(prefix, "f32", 3U) == 0) ||
							(std::strncmp(prefix, "s16", 3U) == 0) ||
							(std::strncmp(prefix, "s32", 3U) == 0);
					default:
						return false;
					}
				}

				inline uint32_t SchemaFieldWidth(const char* prefix, std::size_t length) noexcept
				{
					if (length == 2U)
					{
						return 1U;
					}

					if (length == 3U &&
						((std::strncmp(prefix, "u16", 3U) == 0) ||
						 (std::strncmp(prefix, "i16", 3U) == 0) ||
						 (std::strncmp(prefix, "s16", 3U) == 0)))
					{
						return 2U;
					}

					return 4U;
				}

				inline bool SchemaPrefixUsesStringHash(const char* prefix, std::size_t length) noexcept
				{
					return
						((length == 2U) && (std::strncmp(prefix, "s8", 2U) == 0)) ||
						((length == 3U) &&
						 ((std::strncmp(prefix, "s16", 3U) == 0) || (std::strncmp(prefix, "s32", 3U) == 0)));
				}

				inline bool ValidateTypedSchema(
					const char* schemaCsv,
					uint32_t expectedFieldCount,
					uint32_t expectedRowBytes,
					bool& hasStringHashField) noexcept
				{
					hasStringHashField = false;
					if (schemaCsv == nullptr || schemaCsv[0] == '\0')
					{
						return false;
					}

					uint32_t fieldCount = 0U;
					uint32_t rowBytes = 0U;
					const char* fieldStart = schemaCsv;
					for (const char* cursor = schemaCsv;; ++cursor)
					{
						if (*cursor != ',' && *cursor != '\0')
						{
							continue;
						}

						if (cursor == fieldStart)
						{
							return false;
						}

						const char* separator = fieldStart;
						while (separator < cursor && *separator != '_')
						{
							++separator;
						}

						if (separator == fieldStart || separator == cursor)
						{
							return false;
						}

						const std::size_t prefixLength = static_cast<std::size_t>(separator - fieldStart);
						if (!IsSchemaPrefix(fieldStart, prefixLength))
						{
							return false;
						}

						rowBytes += SchemaFieldWidth(fieldStart, prefixLength);
						hasStringHashField = hasStringHashField || SchemaPrefixUsesStringHash(fieldStart, prefixLength);
						++fieldCount;

						if (*cursor == '\0')
						{
							break;
						}
						fieldStart = cursor + 1;
					}

					return (fieldCount == expectedFieldCount) && (rowBytes == expectedRowBytes);
				}

				inline uint32_t PackTextTagOrHash(const char* text) noexcept
				{
					if (text == nullptr || text[0] == '\0')
					{
						return 0u;
					}

					char tag[4] = { '\0', '\0', '\0', '\0' };
					std::size_t length = 0U;
					while (text[length] != '\0')
					{
						if (length < 4U)
						{
							tag[length] = text[length];
						}
						++length;
					}

					if (length <= 4U)
					{
						return mmlog::TAG4(tag[0], tag[1], tag[2], tag[3]);
					}

					return Fnv1a32(text);
				}
			}
		}
	}
}

#endif
