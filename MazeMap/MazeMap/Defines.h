#ifndef DEFINES
#define DEFINES

constexpr float AVG_SPD_WEIGHT = 0.3f;
//#define SMALL_RDD
#define PATH_SIZE 192
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RT2 = 1.414213562f;
constexpr float HALF_RT2 = 0.707106781f;
constexpr float WALL_THICKNESS = 0.012f;
constexpr float MIN_CLEARANCE = 0.012f;


#ifdef _WINDOWS

 #include <string>

 #ifdef MAZEMAP_EXPORTS

 #define IMPORT __declspec(dllimport)
 #define EXPORT __declspec(dllexport)

 #else

  #define IMPORT __declspec(dllexport)
  #define EXPORT __declspec(dllimport)

 #endif
 #include <string>

#else

 #define EXPORT __declspec(dllexport)
 #define IMPORT __declspec(dllimport)



#endif

#endif
