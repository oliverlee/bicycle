/*
 * This member function is defined in a separate source file to allow different
 * optimization options when using clang as it does not allow per function
 * optimizations as does gcc function attributes.
 */
#include <cmath>
#include <tuple>
#include <boost/math/tools/roots.hpp>
#include "bicycle/bicycle.h"
#include "constants.h"

namespace {
    inline model::real_t square(model::real_t x) {
        return x*x;
    }
}

namespace model {

real_t Bicycle::solve_constraint_pitch(real_t roll, real_t steer, real_t guess, size_t max_iterations) const {
    // constraint function generated by script 'generate_pitch.py'.
    static constexpr int digits = std::numeric_limits<real_t>::digits*2/3;
    static constexpr real_t two = static_cast<real_t>(2.0);
    static constexpr real_t one_five = static_cast<real_t>(1.5);
    static constexpr real_t min = -constants::pi/2;
    static constexpr real_t max = constants::pi/2;
    boost::uintmax_t max_it = max_iterations;

    auto constraint_function = [this, roll, steer](real_t pitch)->std::tuple<real_t, real_t> {
        return std::make_tuple(
((m_rf*square(std::cos(pitch))*square(std::cos(roll)) +
(m_d3*std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll))) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::abs(std::cos(roll)) +
std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll)))*(-m_d1*std::abs(std::cos(roll))*std::sin(pitch) +
m_d2*std::abs(std::cos(roll))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll))/(std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) + square(std::cos(pitch))*square(std::cos(roll)))*std::abs(std::cos(roll)))
                ,
((m_rf*square(std::cos(pitch))*square(std::cos(roll)) +
(m_d3*std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll))) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::abs(std::cos(roll)) +
std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll)))*(-m_d1*std::abs(std::cos(roll))*std::sin(pitch) +
m_d2*std::abs(std::cos(roll))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll))*((-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(pitch)*std::cos(pitch)*square(std::cos(roll)))/(std::pow(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll)), one_five)*std::abs(std::cos(roll))) +
((-m_d1*std::abs(std::cos(roll))*std::cos(pitch) -
m_d2*std::abs(std::cos(roll))*std::sin(pitch))*std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) + square(std::cos(pitch))*square(std::cos(roll)))*std::cos(roll) +
(-(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) -
std::sin(pitch)*std::cos(pitch)*square(std::cos(roll)))*(-m_d1*std::abs(std::cos(roll))*std::sin(pitch) +
m_d2*std::abs(std::cos(roll))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll)/std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) + square(std::cos(pitch))*square(std::cos(roll))) +
(-two*m_rf*std::sin(pitch)*std::cos(pitch)*square(std::cos(roll)) -
(m_d3*std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll))) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::cos(pitch)*std::cos(roll)*std::cos(steer) +
(m_d3*(-(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) -
std::sin(pitch)*std::cos(pitch)*square(std::cos(roll)))/std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll))) -
m_rf*std::cos(pitch)*std::cos(roll)*std::cos(steer))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::abs(std::cos(roll)))/(std::sqrt(square(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)) +
square(std::cos(pitch))*square(std::cos(roll)))*std::abs(std::cos(roll)))
                );
    };
    return boost::math::tools::newton_raphson_iterate(constraint_function, guess, min, max, digits, max_it);
}

} // namespace model
