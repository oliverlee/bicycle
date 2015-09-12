#include <array>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include "bicycle.h"
#include "lqr.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples
    const size_t n = 100;  // length of horizon in samples

    const double as_radians = boost::math::constants::degree<double>(); // convert degrees to radians
    const double as_degrees = boost::math::constants::radian<double>(); // convert radians to degrees

    model::Bicycle::state_t x(
            (model::Bicycle::state_t() <<
                0, 10, 10, 0
                ).finished() * as_radians
            );

    model::Bicycle::state_t xt(
            model::Bicycle::state_t::Zero()
            );

    const controller::Lqr<model::Bicycle>::state_cost_t Q(
            controller::Lqr<model::Bicycle>::state_cost_t::Identity()
            );

    const controller::Lqr<model::Bicycle>::input_cost_t R(
            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity()
            );

    std::array<model::Bicycle::state_t, N> system_state;
    std::array<model::Bicycle::input_t, N> system_control_input;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;

    model::Bicycle bicycle(argv[1], v0, dt);
    controller::Lqr<model::Bicycle> lqr(bicycle, Q, R, xt, n);

    std::cout << "initial state: [" << x.transpose() * as_degrees << "]' deg" << std::endl;

    auto it_x = system_state.begin();
    auto it_u = system_control_input.begin();

    std::cout << std::endl << "simulating..." << std::endl;
    for (; it_x != system_state.end(); ++it_x, ++it_u) {
        *it_u = lqr.control_calculate(x);
        x = bicycle.x_next(x, *it_u);
        *it_x = x;
//        std::cout << x.transpose() * as_degrees << std::endl;
    }

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state: [" << x.transpose() * as_degrees << "]' deg" << std::endl;
    std::cout << "final control input: [" << system_control_input.back().transpose() << "]'" << std::endl;

    return EXIT_SUCCESS;
}
