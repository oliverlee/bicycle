#pragma once

namespace model {
#if BICYCLE_USE_DOUBLE_PRECISION_REAL
using real_t = double;
#else
using real_t = float;
#endif
} // namespace model
