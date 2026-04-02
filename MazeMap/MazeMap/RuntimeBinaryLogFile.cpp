#include "pch.h"
#include "RuntimeBinaryLogFile.h"

#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdio.h>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				RuntimeBinaryLogFile::RuntimeBinaryLogFile() noexcept
					: _file()
					, _sidecarFile()
					, _blocks(nullptr)
					, _activeIndex(-1)
					, _fieldCount(0U)
					, _recordBytes(0U)
					, _blockBytes(0U)
					, _bufferCount(0U)
					, _writeMode(kDefaultRuntimeBinaryLogWriteMode)
					, _isOpen(false)
					, _overflowed(false)
					, _writeFailed(false)
					, _queuedBlockCount(0U)
				{
					_fileName[0] = '\0';
					_sidecarName[0] = '\0';
				}

				RuntimeBinaryLogFile::~RuntimeBinaryLogFile()
				{
					Close();
				}

				bool RuntimeBinaryLogFile::BeginSelected(
					const char* fileName,
					const char* schemaCsv,
					uint32_t fieldCount,
					const char* metadataKv,
					const char* notes,
					uint32_t flags,
					uint32_t runId,
					uint64_t startTimeUs,
					RuntimeBinaryLogWriteMode writeMode,
					std::size_t blockBytes,
					uint8_t bufferCount)
				{
					Close();
					if (fileName == nullptr ||
						fileName[0] == '\0' ||
						schemaCsv == nullptr ||
						schemaCsv[0] == '\0' ||
						fieldCount == 0U)
					{
						return false;
					}

					snprintf(_fileName, sizeof(_fileName), "%s", fileName);
					_fieldCount = fieldCount;
					_recordBytes = fieldCount * sizeof(uint32_t);
					_blockBytes = (writeMode == RuntimeBinaryLogWriteMode::Buffered) ? blockBytes : 0U;
					_bufferCount = (writeMode == RuntimeBinaryLogWriteMode::Buffered) ? bufferCount : 0U;
					_writeMode = writeMode;
					_overflowed = false;
					_writeFailed = false;
					_queuedBlockCount = 0U;

					if ((_writeMode == RuntimeBinaryLogWriteMode::Buffered) && !AllocateBlocks())
					{
						return false;
					}

					if (!_file.Open(_fileName))
					{
						FreeBlocks();
						return false;
					}

					if (!WriteHeaderAndDescriptors(schemaCsv, metadataKv, notes, flags, runId, startTimeUs))
					{
						_file.Close();
						_sidecarFile.Close();
						FreeBlocks();
						_fileName[0] = '\0';
						_sidecarName[0] = '\0';
						return false;
					}

					_isOpen = true;
					return true;
				}

				bool RuntimeBinaryLogFile::AppendRecord(const uint32_t* words, uint32_t fieldCount)
				{
					if (!_isOpen || words == nullptr || fieldCount != _fieldCount)
					{
						return false;
					}

					if (_writeMode == RuntimeBinaryLogWriteMode::Synchronous)
					{
						if (_file.WriteBytes(words, _recordBytes) != _recordBytes)
						{
							_writeFailed = true;
							return false;
						}
						return true;
					}

					if (_blocks == nullptr || _activeIndex < 0)
					{
						return false;
					}

					Block& active = _blocks[_activeIndex];
					if ((active.used + _recordBytes) > _blockBytes)
					{
						if (!EnqueueActiveAndRotate())
						{
							_overflowed = true;
							return false;
						}
					}

					Block& destination = _blocks[_activeIndex];
					std::memcpy(destination.data + destination.used, words, _recordBytes);
					destination.used += _recordBytes;
					return true;
				}

				bool RuntimeBinaryLogFile::Service(uint32_t maxBlocks)
				{
					if (!_isOpen)
					{
						return false;
					}

					if (_writeMode == RuntimeBinaryLogWriteMode::Synchronous)
					{
						return true;
					}

					const uint32_t limit = (maxBlocks == 0U) ? 1U : maxBlocks;
					for (uint32_t index = 0U; index < limit; ++index)
					{
						const int queuedIndex = FindQueuedBlockIndex();
						if (queuedIndex < 0)
						{
							break;
						}
						if (!WriteBlock(queuedIndex))
						{
							return false;
						}
					}
					return true;
				}

				void RuntimeBinaryLogFile::Flush()
				{
					if (!_isOpen)
					{
						return;
					}

					if (_writeMode == RuntimeBinaryLogWriteMode::Buffered)
					{
						while (FindQueuedBlockIndex() >= 0)
						{
							if (!Service(static_cast<uint32_t>(_bufferCount)))
							{
								break;
							}
						}
						(void)WriteActivePartialBlock();
					}

					_file.Flush();
					_sidecarFile.Flush();
				}

				void RuntimeBinaryLogFile::Close()
				{
					if (_isOpen)
					{
						Flush();
						_file.Close();
						_sidecarFile.Close();
					}
					else
					{
						_file.Close();
						_sidecarFile.Close();
					}

					_isOpen = false;
					_fieldCount = 0U;
					_recordBytes = 0U;
					_blockBytes = 0U;
					_bufferCount = 0U;
					_writeMode = kDefaultRuntimeBinaryLogWriteMode;
					_overflowed = false;
					_writeFailed = false;
					_queuedBlockCount = 0U;
					_fileName[0] = '\0';
					_sidecarName[0] = '\0';
					FreeBlocks();
				}

				const char* RuntimeBinaryLogFile::GetFileName() const
				{
					return _fileName;
				}

				const char* RuntimeBinaryLogFile::GetFileName()
				{
					return const_cast<RuntimeBinaryLogFile const*>(this)->GetFileName();
				}

				const char* RuntimeBinaryLogFile::GetSidecarFileName() const
				{
					return _sidecarName;
				}

				const char* RuntimeBinaryLogFile::GetSidecarFileName()
				{
					return const_cast<RuntimeBinaryLogFile const*>(this)->GetSidecarFileName();
				}

				bool RuntimeBinaryLogFile::IsOpen() const
				{
					return _isOpen;
				}

				bool RuntimeBinaryLogFile::IsOpen()
				{
					return const_cast<RuntimeBinaryLogFile const*>(this)->IsOpen();
				}

				bool RuntimeBinaryLogFile::HadOverflow() const
				{
					return _overflowed;
				}

				bool RuntimeBinaryLogFile::HadOverflow()
				{
					return const_cast<RuntimeBinaryLogFile const*>(this)->HadOverflow();
				}

				bool RuntimeBinaryLogFile::HadWriteFailure() const
				{
					return _writeFailed;
				}

				bool RuntimeBinaryLogFile::HadWriteFailure()
				{
					return const_cast<RuntimeBinaryLogFile const*>(this)->HadWriteFailure();
				}

				uint32_t RuntimeBinaryLogFile::InternLabel(const char* text)
				{
					if (text == nullptr || text[0] == '\0')
					{
						return std::numeric_limits<uint32_t>::max();
					}

					const uint32_t hash = Fnv1a32(text);
					if (_sidecarFile.IsOpen())
					{
						const std::size_t length = std::strlen(text);
						if ((_sidecarFile.WriteBytes(text, length) != length) ||
							(_sidecarFile.WriteBytes("\n", 1U) != 1U))
						{
							_writeFailed = true;
						}
					}
					return hash;
				}

				bool RuntimeBinaryLogFile::AllocateBlocks()
				{
					if (_bufferCount < 2U || _blockBytes < _recordBytes)
					{
						return false;
					}

					_blocks = new Block[_bufferCount];
					if (_blocks == nullptr)
					{
						return false;
					}

					for (uint8_t index = 0U; index < _bufferCount; ++index)
					{
						_blocks[index].data = static_cast<uint8_t*>(std::malloc(_blockBytes));
						if (_blocks[index].data == nullptr)
						{
							FreeBlocks();
							return false;
						}
						_blocks[index].used = 0U;
						_blocks[index].queued = false;
						_blocks[index].active = false;
					}

					_activeIndex = 0;
					_blocks[_activeIndex].active = true;
					return true;
				}

				void RuntimeBinaryLogFile::FreeBlocks()
				{
					if (_blocks != nullptr)
					{
						for (uint8_t index = 0U; index < _bufferCount; ++index)
						{
							if (_blocks[index].data != nullptr)
							{
								std::free(_blocks[index].data);
								_blocks[index].data = nullptr;
							}
						}

						delete[] _blocks;
						_blocks = nullptr;
					}

					_activeIndex = -1;
				}

				bool RuntimeBinaryLogFile::WriteHeaderAndDescriptors(
					const char* schemaCsv,
					const char* metadataKv,
					const char* notes,
					uint32_t flags,
					uint32_t runId,
					uint64_t startTimeUs)
				{
					bool hasStringHashField = false;
					if (!ValidateTypedSchema(schemaCsv, _fieldCount, _recordBytes, hasStringHashField))
					{
						return false;
					}

					if (!BuildSiblingRuntimeFileName(_sidecarName, sizeof(_sidecarName), _fileName, ".sidecar"))
					{
						return false;
					}
					if (!_sidecarFile.Open(_sidecarName))
					{
						return false;
					}

					if (!WriteSidecarLine("schema_version=2"))
					{
						return false;
					}
					if (!WriteUnsignedMetadataLine("row_bytes", static_cast<unsigned long>(_recordBytes)))
					{
						return false;
					}
					if (hasStringHashField && !WriteSidecarLine("string_hash=fnv1a32"))
					{
						return false;
					}
					if ((flags != 0U) && !WriteUnsignedMetadataLine("legacy_flags", static_cast<unsigned long>(flags)))
					{
						return false;
					}
					if ((runId != 0U) && !WriteUnsignedMetadataLine("run_id", static_cast<unsigned long>(runId)))
					{
						return false;
					}
					if ((startTimeUs != 0U) && !WriteUnsigned64MetadataLine("start_time_us", startTimeUs))
					{
						return false;
					}
					if (!WriteSidecarBlock(metadataKv) || !WriteSidecarBlock(notes))
					{
						return false;
					}
					if (!WriteSidecarLine(schemaCsv) || !WriteSidecarLine("LABELS:"))
					{
						return false;
					}

					char bindingLine[96] = {};
					const int bindingLength = snprintf(
						bindingLine,
						sizeof(bindingLine),
						"sidecar_file=%s\n",
						FileNameComponent(_sidecarName));
					if (bindingLength <= 0)
					{
						return false;
					}
					return
						_file.WriteBytes(bindingLine, static_cast<std::size_t>(bindingLength)) ==
						static_cast<std::size_t>(bindingLength);
				}

				bool RuntimeBinaryLogFile::WriteSidecarLine(const char* line)
				{
					if (line == nullptr)
					{
						return false;
					}

					const std::size_t length = std::strlen(line);
					return
						(_sidecarFile.WriteBytes(line, length) == length) &&
						(_sidecarFile.WriteBytes("\n", 1U) == 1U);
				}

				bool RuntimeBinaryLogFile::WriteSidecarBlock(const char* block)
				{
					if (block == nullptr || block[0] == '\0')
					{
						return true;
					}

					const std::size_t length = std::strlen(block);
					if (_sidecarFile.WriteBytes(block, length) != length)
					{
						return false;
					}

					if (block[length - 1U] != '\n' && (_sidecarFile.WriteBytes("\n", 1U) != 1U))
					{
						return false;
					}
					return true;
				}

				bool RuntimeBinaryLogFile::WriteUnsignedMetadataLine(const char* key, unsigned long value)
				{
					char line[96] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s=%lu",
						(key != nullptr) ? key : "",
						value);
					if (length <= 0 || length >= static_cast<int>(sizeof(line)))
					{
						return false;
					}
					return WriteSidecarLine(line);
				}

				bool RuntimeBinaryLogFile::WriteUnsigned64MetadataLine(const char* key, uint64_t value)
				{
					char line[96] = {};
					const int length = snprintf(
						line,
						sizeof(line),
						"%s=%llu",
						(key != nullptr) ? key : "",
						static_cast<unsigned long long>(value));
					if (length <= 0 || length >= static_cast<int>(sizeof(line)))
					{
						return false;
					}
					return WriteSidecarLine(line);
				}

				bool RuntimeBinaryLogFile::EnqueueActiveAndRotate()
				{
					if (_blocks == nullptr || _activeIndex < 0)
					{
						return false;
					}

					Block& active = _blocks[_activeIndex];
					if (active.used == 0U)
					{
						return true;
					}

					const int freeIndex = FindFreeBlockIndex();
					if (freeIndex < 0)
					{
						return false;
					}

					active.active = false;
					active.queued = true;
					++_queuedBlockCount;

					_activeIndex = freeIndex;
					_blocks[_activeIndex].active = true;
					_blocks[_activeIndex].queued = false;
					_blocks[_activeIndex].used = 0U;
					return true;
				}

				int RuntimeBinaryLogFile::FindFreeBlockIndex() const noexcept
				{
					if (_blocks == nullptr)
					{
						return -1;
					}

					for (uint8_t index = 0U; index < _bufferCount; ++index)
					{
						if (!_blocks[index].active && !_blocks[index].queued && (_blocks[index].used == 0U))
						{
							return static_cast<int>(index);
						}
					}
					return -1;
				}

				int RuntimeBinaryLogFile::FindQueuedBlockIndex() const noexcept
				{
					if (_blocks == nullptr)
					{
						return -1;
					}

					for (uint8_t index = 0U; index < _bufferCount; ++index)
					{
						if (_blocks[index].queued && !_blocks[index].active)
						{
							return static_cast<int>(index);
						}
					}
					return -1;
				}

				bool RuntimeBinaryLogFile::WriteBlock(int blockIndex)
				{
					if (_blocks == nullptr || blockIndex < 0)
					{
						return false;
					}

					Block& block = _blocks[blockIndex];
					if (block.used == 0U)
					{
						block.queued = false;
						return true;
					}

					if (_file.WriteBytes(block.data, block.used) != block.used)
					{
						_writeFailed = true;
						return false;
					}

					block.used = 0U;
					block.queued = false;
					if (_queuedBlockCount > 0U)
					{
						--_queuedBlockCount;
					}
					return true;
				}

				bool RuntimeBinaryLogFile::WriteActivePartialBlock()
				{
					if (_blocks == nullptr || _activeIndex < 0)
					{
						return false;
					}

					Block& active = _blocks[_activeIndex];
					if (active.used == 0U)
					{
						return true;
					}

					if (_file.WriteBytes(active.data, active.used) != active.used)
					{
						_writeFailed = true;
						return false;
					}

					active.used = 0U;
					return true;
				}
			}
		}
	}
}
