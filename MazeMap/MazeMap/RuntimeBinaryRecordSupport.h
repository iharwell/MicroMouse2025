#ifndef RUNTIMEBINARYRECORDSUPPORT_H
#define RUNTIMEBINARYRECORDSUPPORT_H

// Defines the lightweight binary-record helpers shared by runtime MMLOG writers.

#include "RuntimeBinaryLogFile.h"
#include "RuntimeRecordBuilder.h"
#include "RuntimeTextBlockBuilder.h"

#include <cstdint>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				static constexpr uint32_t kRuntimeBinaryLogFlags = mmlog::FLAG_HAS_METADATA | mmlog::FLAG_HAS_NOTES;

				inline bool AppendRuntimeBinaryNotes(RuntimeTextBlockBuilder<512U>& notes, const char* eventFileName)
				{
					if (!notes.AppendKeyValue("format_spec", "micromouse_logging_file_format_rev_g"))
					{
						return false;
					}
					if (!notes.AppendKeyValue("endianness", "little"))
					{
						return false;
					}
					(void)eventFileName;
					return true;
				}

				template <std::size_t N>
				inline bool AppendBinaryRecord(RuntimeBinaryLogFile& log, const RuntimeRecordBuilder<N>& record)
				{
					return record.IsFull() && log.AppendRecord(record.Data(), record.Count());
				}
			}
		}
	}
}

#endif
