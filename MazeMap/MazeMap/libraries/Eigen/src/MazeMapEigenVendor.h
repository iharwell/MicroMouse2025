#pragma once

#if !defined(EIGEN_NO_DEBUG)
#define EIGEN_NO_DEBUG
#endif

#if !defined(EIGEN_DONT_VECTORIZE)
#define EIGEN_DONT_VECTORIZE
#endif

#if !defined(EIGEN_DONT_ALIGN_STATICALLY)
#define EIGEN_DONT_ALIGN_STATICALLY
#endif

#if !defined(EIGEN_MAX_STATIC_ALIGN_BYTES)
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
#endif

#if !defined(EIGEN_UNROLLING_LIMIT)
#define EIGEN_UNROLLING_LIMIT 0
#endif

#ifdef F
#pragma push_macro("F")
#undef F
#define MAZEMAP_RESTORE_EIGEN_F
#endif

#ifdef B1
#pragma push_macro("B1")
#undef B1
#define MAZEMAP_RESTORE_EIGEN_B1
#endif

#ifdef B2
#pragma push_macro("B2")
#undef B2
#define MAZEMAP_RESTORE_EIGEN_B2
#endif

#ifdef B3
#pragma push_macro("B3")
#undef B3
#define MAZEMAP_RESTORE_EIGEN_B3
#endif

#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/Householder"
#include "Eigen/QR"

#ifdef MAZEMAP_RESTORE_EIGEN_B3
#pragma pop_macro("B3")
#undef MAZEMAP_RESTORE_EIGEN_B3
#endif

#ifdef MAZEMAP_RESTORE_EIGEN_B2
#pragma pop_macro("B2")
#undef MAZEMAP_RESTORE_EIGEN_B2
#endif

#ifdef MAZEMAP_RESTORE_EIGEN_B1
#pragma pop_macro("B1")
#undef MAZEMAP_RESTORE_EIGEN_B1
#endif

#ifdef MAZEMAP_RESTORE_EIGEN_F
#pragma pop_macro("F")
#undef MAZEMAP_RESTORE_EIGEN_F
#endif
