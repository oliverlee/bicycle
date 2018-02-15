#include <array>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include "bicycle/whipple.h"
#include "parameters.h"

namespace {
    const double fs = 1000; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model::BicycleWhipple::state_t, N> continuous_time_system_state;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    double v = 5.0;
    model::BicycleWhipple bicycle(v, dt);
    model::BicycleWhipple::state_t x;
    x << 0, 0, 10, 10, 0; // define in degrees
    x *= constants::as_radians;

    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout <<
        "states are: [yaw angle, roll angle, steer angle, roll rate, steer rate]'" <<
        std::endl << std::endl;

    std::cout <<
        "simulating (zero input) continuous time system at constant speed v = " <<
        v << " ..." << std::endl;

    for (auto& state: continuous_time_system_state) {
        state = bicycle.integrate_state(x, model::BicycleWhipple::input_t::Zero(), dt);
        x = state;
    }

    std::cout << std::endl;
    std::cout << "state at end of simulation (" << N << " steps)" << std::endl;
    std::cout << "continuous time (zero input):   " <<
        continuous_time_system_state.back().transpose() << std::endl;
    std::cout << std::endl;

    v = 6.0;
    bicycle.set_v(v);
    x << 0, 0, 10, 10, 0; // define in degrees
    x *= constants::as_radians;

    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout <<
        "states are: [yaw angle, roll angle, steer angle, roll rate, steer rate]'" <<
        std::endl << std::endl;

    std::cout <<
        "simulating (zero input) continuous time system at constant speed v = " <<
        v << " ..." << std::endl;

    for (auto& state: continuous_time_system_state) {
        state = bicycle.integrate_state(x, model::BicycleWhipple::input_t::Zero(), dt);
        x = state;
    }

    std::cout << std::endl;
    std::cout << "state at end of simulation (" << N << " steps)" << std::endl;
    std::cout << "continuous time (zero input):   " <<
        continuous_time_system_state.back().transpose() << std::endl;
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
