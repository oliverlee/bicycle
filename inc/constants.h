#pragma once
#include <boost/math/constants/constants.hpp>

namespace constants {
    constexpr double g = 9.80665; // gravitational constant [m/s^2]
    const double as_radians = boost::math::constants::degree<double>(); // convert degrees to radians
    const double as_degrees = boost::math::constants::radian<double>(); // convert radians to degrees
    const double two_pi = boost::math::constants::two_pi<double>();
    const double pi = boost::math::constants::pi<double>();
} // namespace constants
