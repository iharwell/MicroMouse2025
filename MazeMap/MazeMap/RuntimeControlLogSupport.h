#ifndef RUNTIMECONTROLLOGSUPPORT_H
#define RUNTIMECONTROLLOGSUPPORT_H

// Defines the centralized plain-text runtime control-log helpers shared by event-log writers.

#include "CoreFileExport.h"

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

				inline MazeMap::CoreFileExport& RuntimeControlLogFile() noexcept
				{
					static MazeMap::CoreFileExport file;
					return file;
				}

				inline bool& RuntimeControlLogFaulted() noexcept
				{
					static bool faulted = false;
					return faulted;
				}

				inline bool EnsureRuntimeControlLogOpen()
				{
					if (RuntimeControlLogFaulted())
					{
						return false;
					}

					MazeMap::CoreFileExport& file = RuntimeControlLogFile();
					if (file.IsOpen())
					{
						return true;
					}

					if (!file.Open(kRuntimeControlLogFileName))
					{
						RuntimeControlLogFaulted() = true;
						return false;
					}
					return true;
				}

				inline bool RuntimeControlLogIsOpen()
				{
					return RuntimeControlLogFile().IsOpen();
				}

				inline bool AppendRuntimeControlLogLine(const char* line)
				{
					if (line == nullptr || line[0] == '\0' || !EnsureRuntimeControlLogOpen())
					{
						return false;
					}

					MazeMap::CoreFileExport& file = RuntimeControlLogFile();
					if (!file.Write(line) || !file.WriteChar('\n'))
					{
						file.Close();
						RuntimeControlLogFaulted() = true;
						return false;
					}
					return true;
				}

				inline void FlushRuntimeControlLog()
				{
					if (RuntimeControlLogIsOpen())
					{
						RuntimeControlLogFile().Flush();
					}
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
