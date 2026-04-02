#include "pch.h"
#include "OptionalRuntimeEventLog.h"
#include "RuntimeBinaryLogSupport.h"
#include "RuntimeControlLogSupport.h"

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
				OptionalRuntimeEventLog::OptionalRuntimeEventLog() noexcept
					: _enabled(false)
				{
					_source[0] = '\0';
				}

				bool OptionalRuntimeEventLog::BeginSibling(const char* dataFileName)
				{
					Close();
					const char* component = FileNameComponent(dataFileName);
					if (component == nullptr || component[0] == '\0')
					{
						snprintf(_source, sizeof(_source), "%s", "runtime");
					}
					else
					{
						const char* extension = std::strrchr(component, '.');
						std::size_t sourceLength =
							(extension != nullptr) ? static_cast<std::size_t>(extension - component) : std::strlen(component);
						if (sourceLength >= sizeof(_source))
						{
							sourceLength = sizeof(_source) - 1U;
						}
						std::memcpy(_source, component, sourceLength);
						_source[sourceLength] = '\0';
					}

					_enabled = true;
					return true;
				}

				bool OptionalRuntimeEventLog::WriteMetadata(const char* key, const char* value)
				{
					if (!_enabled)
					{
						return true;
					}

					char message[256] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"%s=%s",
						(key != nullptr) ? key : "",
						(value != nullptr) ? value : "");
					if (length > 0)
					{
						message[sizeof(message) - 1U] = '\0';
						(void)AppendRuntimeControlLogEntry(_source, micros(), "metadata", message);
					}
					return true;
				}

				bool OptionalRuntimeEventLog::WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name)
				{
					if (!_enabled)
					{
						return true;
					}

					char message[256] = {};
					const int length = snprintf(
						message,
						sizeof(message),
						"phase_id=%lu;name=%s",
						phaseId,
						(name != nullptr) ? name : "");
					if (length > 0)
					{
						message[sizeof(message) - 1U] = '\0';
						(void)AppendRuntimeControlLogEntry(_source, timestampUs, "phase", message);
					}
					return true;
				}

				bool OptionalRuntimeEventLog::WriteEvent(unsigned long timestampUs, const char* type, const char* message)
				{
					if (_enabled)
					{
						(void)AppendRuntimeControlLogEntry(_source, timestampUs, type, message);
					}
					return true;
				}

				void OptionalRuntimeEventLog::Flush()
				{
				}

				void OptionalRuntimeEventLog::Close()
				{
					_enabled = false;
					_source[0] = '\0';
				}

				bool OptionalRuntimeEventLog::IsEnabled() const
				{
					return _enabled;
				}

				bool OptionalRuntimeEventLog::IsEnabled()
				{
					return const_cast<OptionalRuntimeEventLog const*>(this)->IsEnabled();
				}

				const char* OptionalRuntimeEventLog::GetFileName() const
				{
					return _enabled ? kRuntimeControlLogFileName : "";
				}

				const char* OptionalRuntimeEventLog::GetFileName()
				{
					return const_cast<OptionalRuntimeEventLog const*>(this)->GetFileName();
				}
			}
		}
	}
}
