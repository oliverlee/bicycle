#include <array>
#include <iostream>
#include <random>
#include "bicycle/kinematic.h"
#include "parameters.h"

namespace {
    using model_t = model::BicycleKinematic;

    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model_t::state_t, N> system_state;
    std::random_device rd; // used to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> r0(0, parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> r1(0, parameters::defaultvalue::kalman::R(1, 1));

    model_t bicycle(v0, dt);
    bicycle.set_v_dt(v0, dt);
    std::cout << "M: " << std::endl << bicycle.M() << std::endl;
    std::cout << "C1: " << std::endl << bicycle.C1() << std::endl;
    std::cout << "K0: " << std::endl << bicycle.K0() << std::endl;
    std::cout << "K2: " << std::endl << bicycle.K2() << std::endl << std::endl;

    model_t::state_t x, x0;
    x0 << 0, 0, 10, 10, 0; // define in degrees
    x0 *= constants::as_radians;

    model_t::measurement_t z;

    std::cout << "initial state: [" << x0.transpose() << "]' rad" << std::endl;
    std::cout << "states are: [yaw angle, roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    std::cout << "simulating simplified kinematic bicycle model at constant speed..." << std::endl;
    x = x0;
    for (auto& state: system_state) {
        model_t::set_output_element(z, model_t::output_index_t::yaw_angle,
                model_t::get_state_element(x, model_t::state_index_t::yaw_angle)  + r0(gen));
        model_t::set_output_element(z, model_t::output_index_t::steer_angle,
                model_t::get_state_element(x, model_t::state_index_t::steer_angle) + r1(gen));

        state = bicycle.update_state(x, model_t::input_t::Zero(), z);
        x = state;
    }

    std::cout << std::endl;
    std::cout << "state at end of simulation (" << N << " steps)" << std::endl;
    std::cout << system_state.back().transpose() << std::endl;
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
