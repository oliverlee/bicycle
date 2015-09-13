#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "lqr.h"
#include "parameters.h"
#include "test_utilities.h"

// TODO: Add more extensive testing
namespace {
    const double reltol = 1e-2;
    const double abstol = 1e-1;
    const double dt = 1.0/200;
    const double v = 4.0;
    const uint32_t N = 1000; // simulation length
    const size_t n = 100;  // length of horizon in samples


    const model::Bicycle::output_matrix_t C = parameters::defaultvalue::bicycle::C;
    model::Bicycle::state_t x( // initial state
            (model::Bicycle::state_t() <<
                0, 10, 10, 0).finished() * constants::as_radians);
    const model::Bicycle::state_t xt( // target state, initial estimator state
            model::Bicycle::state_t::Zero());

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

    std::random_device rd; // used only to seed rng
} // namespace

TEST(LqrKalman, Convergence) {
    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0, R(0, 0));
    std::normal_distribution<> rn1(0, R(1, 1));

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, v, dt);
    bicycle.set_C(C);
    observer::Kalman<model::Bicycle> kalman(bicycle, Qn, Rn, xt, P0);
    controller::Lqr<model::Bicycle> lqr(bicycle, Q, R, xt, n);

    // set initial state
    x << 0, 10, 10, 0; // define x in degrees

    for(unsigned int i = 0; i < N; ++i) {
        // compute control law
        auto u = lqr.control_calculate(kalman.x());
        x = bicycle.x_next(x, u);

        // simulate system
        x = bicycle.x_next(x, u);

        // measure output with noise
        auto z = bicycle.y(x);
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        // observer time/measurement update
        kalman.time_update(u);
        kalman.measurement_update(z);
    }

    EXPECT_TRUE(test::allclose(kalman.x(), x, reltol, abstol)) << output_matrices(kalman.x().transpose(), x.transpose());
}
