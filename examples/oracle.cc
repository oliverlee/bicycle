#include <array>
#include <iostream>
#include <random>
#include "bicycle.h"
#include "oracle.h"
#include "parameters.h"

namespace {
    constexpr double fs = 200; // sample rate [Hz]
    constexpr double dt = 1.0/fs; // sample time [s]
    constexpr double v0 = 4.0; // forward speed [m/s]
    constexpr size_t N = 1000; // length of simulation in samples

    using bicycle_t = model::Bicycle;
    using observer_t = observer::Oracle<bicycle_t>;

    bicycle_t::state_t x;
    observer_t::input_t u;
    observer_t::measurement_t z;
    std::random_device rd; // used only to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> r0(0, 1);
    std::normal_distribution<> r1(0, 1);

    bicycle_t bicycle(v0, dt);
    observer_t oracle(bicycle);

    z.setZero();

    std::cout << "simulating bicycle model with oracle observer " << std::endl;
    for (size_t i = 0; i < N; ++i) {
        u(0) = r0(gen);
        u(1) = r1(gen);
        oracle.update_state(u, z);
    }

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "[" << oracle.state().transpose() * constants::as_degrees << "]' deg" << std::endl;

    return EXIT_SUCCESS;
}
