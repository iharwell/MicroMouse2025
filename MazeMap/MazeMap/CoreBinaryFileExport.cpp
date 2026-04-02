#include "pch.h"
#include "CoreBinaryFileExport.h"

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				CoreBinaryFileExport::CoreBinaryFileExport()
				{
				}

				CoreBinaryFileExport::~CoreBinaryFileExport()
				{
					Close();
				}

				bool CoreBinaryFileExport::Open(const char* fileName)
				{
					Close();
					if (fileName == nullptr || fileName[0] == '\0')
					{
						return false;
					}

#if defined(ARDUINO_TEENSY41)
					SD.remove(fileName);
					_file = SD.open(fileName, FILE_WRITE);
					return static_cast<bool>(_file);
#else
					_file.open(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
					return _file.is_open();
#endif
				}

				bool CoreBinaryFileExport::IsOpen() const
				{
#if defined(ARDUINO_TEENSY41)
					return static_cast<bool>(_file);
#else
					return _file.is_open();
#endif
				}

				bool CoreBinaryFileExport::IsOpen()
				{
					return const_cast<CoreBinaryFileExport const*>(this)->IsOpen();
				}

				std::size_t CoreBinaryFileExport::WriteBytes(const void* data, std::size_t size)
				{
					if (!IsOpen() || data == nullptr)
					{
						return 0U;
					}

					if (size == 0U)
					{
						return 0U;
					}

#if defined(ARDUINO_TEENSY41)
					return _file.write(reinterpret_cast<const uint8_t*>(data), size);
#else
					_file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
					return _file.good() ? size : 0U;
#endif
				}

				void CoreBinaryFileExport::Flush()
				{
					if (!IsOpen())
					{
						return;
					}

					_file.flush();
				}

				void CoreBinaryFileExport::Close()
				{
#if defined(ARDUINO_TEENSY41)
					if (_file)
					{
						_file.close();
					}
#else
					if (_file.is_open())
					{
						_file.close();
					}
#endif
				}
			}
		}
	}
}
