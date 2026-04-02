#ifndef COREBINARYFILEEXPORT_H
#define COREBINARYFILEEXPORT_H

// Declares the low-level binary file handle used by runtime MMLOG streams and sidecar exports.

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

#include <cstddef>

namespace MazeMap
{
	namespace App
	{
		namespace Internal
		{
			namespace Runtime
			{
				// Owns the on-disk file used by runtime binary log writers and their sidecar metadata files.
				class CoreBinaryFileExport
				{
				private:
#if defined(ARDUINO_TEENSY41)
					File _file;
#else
					std::ofstream _file;
#endif

				public:
					CoreBinaryFileExport();
					~CoreBinaryFileExport();

					bool Open(const char* fileName);

					// Returns true if the binary export currently owns an open file handle.
					bool IsOpen();
					// Returns true if the binary export currently owns an open file handle.
					bool IsOpen() const;

					std::size_t WriteBytes(const void* data, std::size_t size);
					void Flush();
					void Close();
				};
			}
		}
	}
}

#endif
