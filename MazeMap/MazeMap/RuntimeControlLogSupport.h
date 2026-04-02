#ifndef RUNTIMECONTROLLOGSUPPORT_H
#define RUNTIMECONTROLLOGSUPPORT_H

// Defines the plain-text runtime control-log helpers shared by event-log writers.

#include "Defines.h"

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

#include <stdio.h>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				inline constexpr const char* kRuntimeControlLogFileName = "logging.txt";

				inline bool AppendRuntimeControlLogLine(const char* line)
				{
					if (line == nullptr || line[0] == '\0')
					{
						return false;
					}

#if defined(ARDUINO_TEENSY41)
					File file = SD.open(kRuntimeControlLogFileName, FILE_WRITE);
					if (!file)
					{
						return false;
					}

					const bool ok = (file.print(line) > 0U) && (file.write('\n') == 1U);
					file.flush();
					file.close();
					return ok;
#else
					std::ofstream file(kRuntimeControlLogFileName, std::ios::out | std::ios::app);
					if (!file.is_open())
					{
						return false;
					}

					file << line << '\n';
					file.flush();
					return file.good();
#endif
				}

				inline bool AppendRuntimeControlLogEntry(
					const char* source,
					unsigned long timestampUs,
					const char* type,
					const char* message)
				{
					char line[384] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s [%lu] %s%s%s",
						(source != nullptr && source[0] != '\0') ? source : "runtime",
						timestampUs,
						(type != nullptr && type[0] != '\0') ? type : "event",
						(message != nullptr && message[0] != '\0') ? ": " : "",
						(message != nullptr) ? message : "");
					if (length <= 0)
					{
						return false;
					}

					line[sizeof(line) - 1U] = '\0';
					return AppendRuntimeControlLogLine(line);
				}
			}
		}
	}
}

#endif
