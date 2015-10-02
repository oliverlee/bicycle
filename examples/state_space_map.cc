#include <iostream>
#include <utility>
#include "bicycle.h"
#include "parameters.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double vw = 4.29238253634111; // forward speed [m/s]
    const double vc = 6.02426201538837; // forward speed [m/s]

    // These matrices are (obviously) not correct and are used only to
    // determine if discrete state space matrices are correctly looked up.
    const model::Bicycle::state_matrix_t Ad43(
            2 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd43(
            3 * model::Bicycle::input_matrix_t::Identity()
            );
    const model::Bicycle::state_matrix_t Ad60(
            4 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd60(
            5 * model::Bicycle::input_matrix_t::Identity()
            );

    const model::Bicycle::state_space_map_t state_space_map {
        {model::Bicycle::make_state_space_map_key(vw, dt),
            model::Bicycle::state_space_map_value_t(Ad43, Bd43)},
        {model::Bicycle::make_state_space_map_key(vc, dt),
            model::Bicycle::state_space_map_value_t(Ad60, Bd60)},
    };
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::Bicycle bicycle0(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt, &state_space_map);
    model::Bicycle bicycle1(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt);

    std::cout << "for vw: " << vw << ", dt: " << dt << std::endl;
    std::cout << "with discrete state space map, Ad" << std::endl <<
        bicycle0.Ad() << std::endl;
    std::cout << "without discrete state space map, Ad" << std::endl <<
        bicycle1.Ad() << std::endl;

    std::cout << std::endl << "changing parameters..." << std::endl;
    bicycle0.set_v(vc, dt);
    bicycle1.set_v(vc, dt);
    std::cout << "for vc: " << vc << ", dt: " << dt << std::endl;
    std::cout << "with discrete state space map, Ad" << std::endl <<
        bicycle0.Ad() << std::endl;
    std::cout << "without discrete state space map, Ad" << std::endl <<
        bicycle1.Ad() << std::endl;

    std::cout << std::endl << "changing parameters..." << std::endl;
    bicycle0.set_v(5.0, dt);
    bicycle1.set_v(5.0, dt);
    std::cout << "for v: 5.0, dt: " << dt << std::endl;
    std::cout << "with discrete state space map, Ad" << std::endl <<
        bicycle0.Ad() << std::endl;
    std::cout << "without discrete state space map, Ad" << std::endl <<
        bicycle1.Ad() << std::endl;

    return EXIT_SUCCESS;
}
