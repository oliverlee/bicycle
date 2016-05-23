#pragma once
#include <Eigen/Dense>

namespace model {
#if defined(BICYCLE_USE_DOUBLE_PRECISION_REAL)
using real_t = double;
#else
using real_t = float;
#endif
} // namespace model
