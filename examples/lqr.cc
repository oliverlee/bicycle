#include <array>
#include <chrono>
#include <iostream>
#include "bicycle.h"
#include "lqr.h"
#include "parameters.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples
    const size_t n = 100;  // length of horizon in samples

    const model::Bicycle::state_t x0(
            (model::Bicycle::state_t() <<
                0, 10, 10, 0).finished() * constants::as_radians);
    model::Bicycle::state_t x;

    std::array<model::Bicycle::state_t, N> system_state;
    std::array<model::Bicycle::input_t, N> system_control_input;

    std::chrono::time_point<std::chrono::system_clock> start, stop;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v0, dt);
    controller::Lqr<model::Bicycle> lqr(bicycle,
            controller::Lqr<model::Bicycle>::state_cost_t::Identity(),
            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity(),
            model::Bicycle::state_t::Zero(), n);

    x = x0;
    std::cout << "initial state: [" << x.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << std::endl << "simulating without controller..." << std::endl;
    auto it_x = system_state.begin();
    start = std::chrono::system_clock::now();
    for (; it_x != system_state.end(); ++it_x) {
        x = bicycle.x_next(x);
        *it_x = x;
    }
    stop = std::chrono::system_clock::now();
    auto duration = stop - start;
    std::cout << "duration for simulation without controller: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count() <<
        " us" << std::endl;

    x = x0;
    std::cout << std::endl << "simulating with controller..." << std::endl;
    it_x = system_state.begin();
    auto it_u = system_control_input.begin();
    start = std::chrono::system_clock::now();
    for (; it_x != system_state.end(); ++it_x, ++it_u) {
        *it_u = lqr.control_calculate(x);
        x = bicycle.x_next(x, *it_u);
        *it_x = x;
    }
    stop = std::chrono::system_clock::now();
    duration = stop - start;
    std::cout << "duration for simulation with controller: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count() <<
        " us" << std::endl;

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state: [" << system_state.back().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "final control input: [" << system_control_input.back().transpose() << "]'" << std::endl;

    return EXIT_SUCCESS;
}
