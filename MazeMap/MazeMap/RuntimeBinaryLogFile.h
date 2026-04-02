#ifndef RUNTIMEBINARYLOGFILE_H
#define RUNTIMEBINARYLOGFILE_H

// Declares the MMLOG-compatible runtime binary stream writer used by MazeMap diagnostics and measurements.

#include "CoreBinaryFileExport.h"
#include "RuntimeBinaryLogSupport.h"

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
				// Writes fixed-width runtime telemetry rows into a binary log file with a matching sidecar description.
				class EXPORT RuntimeBinaryLogFile
				{
				private:
					struct Block
					{
						uint8_t* data = nullptr;
						uint32_t used = 0U;
						bool queued = false;
						bool active = false;
					};

					CoreBinaryFileExport _file;
					CoreBinaryFileExport _sidecarFile;
					Block* _blocks;
					int _activeIndex;
					uint32_t _fieldCount;
					uint32_t _recordBytes;
					std::size_t _blockBytes;
					uint8_t _bufferCount;
					RuntimeBinaryLogWriteMode _writeMode;
					bool _isOpen;
					bool _overflowed;
					bool _writeFailed;
					uint32_t _queuedBlockCount;
					char _fileName[64];
					char _sidecarName[64];

					bool AllocateBlocks();
					void FreeBlocks();
					bool WriteHeaderAndDescriptors(
						const char* schemaCsv,
						const char* metadataKv,
						const char* notes,
						uint32_t flags,
						uint32_t runId,
						uint64_t startTimeUs);
					bool WriteSidecarLine(const char* line);
					bool WriteSidecarBlock(const char* block);
					bool WriteUnsignedMetadataLine(const char* key, unsigned long value);
					bool WriteUnsigned64MetadataLine(const char* key, uint64_t value);
					bool EnqueueActiveAndRotate();
					int FindFreeBlockIndex() const noexcept;
					int FindQueuedBlockIndex() const noexcept;
					bool WriteBlock(int blockIndex);
					bool WriteActivePartialBlock();

				public:
					RuntimeBinaryLogFile() noexcept;
					~RuntimeBinaryLogFile();

					bool BeginSelected(
						const char* fileName,
						const char* schemaCsv,
						uint32_t fieldCount,
						const char* metadataKv,
						const char* notes,
						uint32_t flags = mmlog::FLAG_HAS_METADATA | mmlog::FLAG_HAS_SEQ | mmlog::FLAG_HAS_T_US,
						uint32_t runId = 0U,
						uint64_t startTimeUs = 0U,
						RuntimeBinaryLogWriteMode writeMode = kDefaultRuntimeBinaryLogWriteMode,
						std::size_t blockBytes = 4096U,
						uint8_t bufferCount = 4U);

					bool AppendRecord(const uint32_t* words, uint32_t fieldCount);
					bool Service(uint32_t maxBlocks = 1U);
					void Flush();
					void Close();

					// Gets the primary MMLOG file name currently owned by the logger.
					const char* GetFileName();
					// Gets the primary MMLOG file name currently owned by the logger.
					const char* GetFileName() const;

					// Gets the sidecar metadata file name currently owned by the logger.
					const char* GetSidecarFileName();
					// Gets the sidecar metadata file name currently owned by the logger.
					const char* GetSidecarFileName() const;

					bool IsOpen();
					bool IsOpen() const;
					bool HadOverflow();
					bool HadOverflow() const;
					bool HadWriteFailure();
					bool HadWriteFailure() const;

					uint32_t InternLabel(const char* text);
				};
			}
		}
	}
}

#endif
