#ifndef OPTIONALRUNTIMEEVENTLOG_H
#define OPTIONALRUNTIMEEVENTLOG_H

// Declares the runtime event log shim that mirrors structured metadata and events into the control log.

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Provides an optional control-log sink for runtime metadata, phase markers, and fault messages.
				class OptionalRuntimeEventLog
				{
				private:
					bool _enabled;
					char _source[64];

				public:
					OptionalRuntimeEventLog() noexcept;

					bool BeginSibling(const char* dataFileName);
					bool WriteMetadata(const char* key, const char* value);
					bool WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name);
					bool WriteEvent(unsigned long timestampUs, const char* type, const char* message);
					void Flush();
					void Close();

					bool IsEnabled();
					bool IsEnabled() const;

					// Gets the control-log file name when event logging is enabled.
					const char* GetFileName();
					// Gets the control-log file name when event logging is enabled.
					const char* GetFileName() const;
				};
			}
		}
	}
}

#endif
