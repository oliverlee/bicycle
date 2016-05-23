#pragma once
#include <boost/math/constants/constants.hpp>
#include "types.h"

namespace constants {
    constexpr model::real_t g = 9.80665; // gravitational constant [m/s^2]
    const model::real_t as_radians = boost::math::constants::degree<model::real_t>(); // convert degrees to radians
    const model::real_t as_degrees = boost::math::constants::radian<model::real_t>(); // convert radians to degrees
    const model::real_t two_pi = boost::math::constants::two_pi<model::real_t>();
    const model::real_t pi = boost::math::constants::pi<model::real_t>();
} // namespace constants
