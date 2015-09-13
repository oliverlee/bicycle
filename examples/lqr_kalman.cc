#include <array>
#include <iostream>
#include <random>
#include "bicycle.h"
#include "lqr.h"
#include "kalman.h"
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

    model::Bicycle::state_t xt(
            model::Bicycle::state_t::Zero());

    const model::Bicycle::output_matrix_t C = parameters::defaultvalue::bicycle::C;

    const controller::Lqr<model::Bicycle>::state_cost_t Q(
            controller::Lqr<model::Bicycle>::state_cost_t::Identity());

    const controller::Lqr<model::Bicycle>::input_cost_t R(
            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity());

    const observer::Kalman<model::Bicycle>::error_covariance_t P0 =
        parameters::defaultvalue::kalman::P0;
    const observer::Kalman<model::Bicycle>::process_noise_covariance_t Qn =
        parameters::defaultvalue::kalman::Q;
    const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t Rn =
        parameters::defaultvalue::kalman::R;

    std::array<model::Bicycle::state_t, N> system_state;
    std::array<model::Bicycle::state_t, N> system_state_estimate;

    std::random_device rd; // used only to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;

    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0, Rn(0, 0));
    std::normal_distribution<> rn1(0, Rn(1, 1));

    model::Bicycle bicycle(argv[1], v0, dt);
    controller::Lqr<model::Bicycle> lqr(bicycle, Q, R, xt, n);
    observer::Kalman<model::Bicycle> kalman(bicycle, Qn, Rn, xt, P0);

    bicycle.set_C(C);

    std::cout << "initial state:          [" << x0.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;

    std::cout << std::endl << "simulating with observer and controller..." << std::endl;

    auto it_x = system_state.begin();
    auto it_xh =  system_state_estimate.begin();
    *it_x++ = x0;
    *it_xh++ = xt;

    auto x = x0;
    model::Bicycle::input_t u;

    for (; it_x != system_state.end(); ++it_x, ++it_xh) {
        // compute control law
        u = lqr.control_calculate(kalman.x());

        // system simulate
        x = bicycle.x_next(x, u);

        // measure output with noise
        auto y = bicycle.y(x);
        auto z = y;
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        // observer time/measurement update
        kalman.time_update(u);
        kalman.measurement_update(z);

        auto error = x - kalman.x();
        std::cout << error.transpose() * constants::as_degrees << std::endl;
    }

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state:          [" << system_state.back().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "final state estimate: [" << system_state_estimate.back().transpose() * constants::as_degrees << "]'" << std::endl;

    return EXIT_SUCCESS;
}
