#pragma once

// Arduino core headers define short macros such as F/B1/B2/B3 that collide
// with Eigen identifiers. Strip them only while parsing Eigen headers.
#if defined(ARDUINO)
#ifndef EIGEN_DONT_PARALLELIZE
#define EIGEN_DONT_PARALLELIZE
#endif
#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#ifndef EIGEN_NO_DEBUG
#define EIGEN_NO_DEBUG
#endif
#ifndef EIGEN_UNROLLING_LIMIT
#define EIGEN_UNROLLING_LIMIT 0
#endif
#if defined(F)
#pragma push_macro("F")
#undef F
#define MAZEMAP_EIGEN_RESTORE_F 1
#endif
#if defined(B1)
#pragma push_macro("B1")
#undef B1
#define MAZEMAP_EIGEN_RESTORE_B1 1
#endif
#if defined(B2)
#pragma push_macro("B2")
#undef B2
#define MAZEMAP_EIGEN_RESTORE_B2 1
#endif
#if defined(B3)
#pragma push_macro("B3")
#undef B3
#define MAZEMAP_EIGEN_RESTORE_B3 1
#endif
#endif

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Householder>
#include <Eigen/QR>

#if defined(MAZEMAP_EIGEN_RESTORE_B3)
#pragma pop_macro("B3")
#undef MAZEMAP_EIGEN_RESTORE_B3
#endif
#if defined(MAZEMAP_EIGEN_RESTORE_B2)
#pragma pop_macro("B2")
#undef MAZEMAP_EIGEN_RESTORE_B2
#endif
#if defined(MAZEMAP_EIGEN_RESTORE_B1)
#pragma pop_macro("B1")
#undef MAZEMAP_EIGEN_RESTORE_B1
#endif
#if defined(MAZEMAP_EIGEN_RESTORE_F)
#pragma pop_macro("F")
#undef MAZEMAP_EIGEN_RESTORE_F
#endif
