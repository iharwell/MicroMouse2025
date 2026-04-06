#ifndef RUNTIMEBINARYLOGSUPPORT_H
#define RUNTIMEBINARYLOGSUPPORT_H

// Defines shared mmlog support utilities for runtime file naming and metadata formatting.

#include "MazeMapRuntimeMmLog.h"

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

				inline bool WriteMmLogMetadataUnsigned(
					MazeMap::mmlog::MmLogLogger& log,
					const char* key,
					unsigned long value)
				{
					if (key == nullptr || key[0] == '\0')
					{
						return false;
					}

					char valueText[32] = {};
					const int length = snprintf(valueText, sizeof(valueText), "%lu", value);
					if (length <= 0 || length >= static_cast<int>(sizeof(valueText)))
					{
						return false;
					}
					return log.writeMetadata(key, valueText);
				}

				inline bool WriteMmLogMetadataFloat(
					MazeMap::mmlog::MmLogLogger& log,
					const char* key,
					float value,
					std::uint8_t precision)
				{
					if (key == nullptr || key[0] == '\0')
					{
						return false;
					}

					char valueText[48] = {};
					const int length = snprintf(
						valueText,
						sizeof(valueText),
						"%.*f",
						static_cast<int>(precision),
						value);
					if (length <= 0 || length >= static_cast<int>(sizeof(valueText)))
					{
						return false;
					}
					return log.writeMetadata(key, valueText);
				}

				inline void CaptureMmLogFailure(
					const MazeMap::mmlog::MmLogLogger& log,
					bool& overflowed,
					bool& writeFailed) noexcept
				{
					const char* error = log.lastError();
					if (error == nullptr || error[0] == '\0')
					{
						return;
					}

					if (std::strstr(error, "overflow") != nullptr)
					{
						overflowed = true;
					}
					else
					{
						writeFailed = true;
					}
				}
			}
		}
	}
}

#endif
