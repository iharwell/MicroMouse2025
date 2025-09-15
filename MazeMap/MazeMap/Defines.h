#ifndef DEFINES
#define DEFINES

#ifdef _WINDOWS

#define IMPORT __declspec(dllimport)
#define EXPORT __declspec(dllexport)

#include <string>

#else

#define EXPORT __declspec(dllexport)
#define IMPORT __declspec(dllimport)

#endif

#endif
