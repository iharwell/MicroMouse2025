#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Eigen\\src\\Core\\arch\\LSX\\GeneralBlockPanelKernel.h"
// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

#ifndef EIGEN_LSX_GEBP_NR
#define EIGEN_LSX_GEBP_NR 8
#endif

template <>
struct gebp_traits<float, float, false, false, Architecture::LSX, GEBPPacketFull>
    : gebp_traits<float, float, false, false, Architecture::Generic, GEBPPacketFull> {
  enum { nr = EIGEN_LSX_GEBP_NR };
};

template <>
struct gebp_traits<double, double, false, false, Architecture::LSX, GEBPPacketFull>
    : gebp_traits<double, double, false, false, Architecture::Generic, GEBPPacketFull> {
  enum { nr = EIGEN_LSX_GEBP_NR };
};
}  // namespace internal
}  // namespace Eigen
