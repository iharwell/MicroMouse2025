#ifndef DEFINES
#define DEFINES

#define PATH_SIZE 128
#define PI_F 3.14159265358979323846f

#ifdef _WINDOWS
#ifdef MAZEMAP_EXPORTS
#define IMPORT __declspec(dllimport)
#define EXPORT __declspec(dllexport)
#else
#define IMPORT __declspec(dllexport)
#define EXPORT __declspec(dllimport)

#endif
#include <string>

time_t GetPreciseTime();

#else

#define EXPORT __declspec(dllexport)
#define IMPORT __declspec(dllimport)



#endif

#endif
