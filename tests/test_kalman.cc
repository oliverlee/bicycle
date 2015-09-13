#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"
#include "test_utilities.h"

// TODO: Add more extensive testing
namespace {
    const double reltol = 1e-4;
    const double abstol = 1e-8;
    const double dt = 1.0/200;
    const double v = 4.0;
    const uint32_t N = 1000; // simulation length

    model::Bicycle::state_t x(
            model::Bicycle::state_t::Zero());

    const model::Bicycle::output_matrix_t C = parameters::defaultvalue::bicycle::C;

    const observer::Kalman<model::Bicycle>::error_covariance_t P0 =
        parameters::defaultvalue::kalman::P0;
    const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q =
        parameters::defaultvalue::kalman::Q;
    const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R =
        parameters::defaultvalue::kalman::R;

    std::random_device rd; // used only to seed rng
} // namespace

TEST(Kalman, Convergence) {
    std::mt19937 gen(rd());
    std::normal_distribution<> r0(0, R(0, 0));
    std::normal_distribution<> r1(0, R(1, 1));

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, v, dt);
    bicycle.set_C(C);
    // set x estimate to zero in Kalman filter
    observer::Kalman<model::Bicycle> kalman(bicycle, Q, R, x, P0);

    // set initial state
    x << 0, 10, 10, 0; // define x in degrees

    for(unsigned int i = 0; i < N; ++i) {
        x = bicycle.x_next(x);

        auto z = bicycle.y(x);
        z(0) += r0(gen);
        z(1) += r1(gen);

        kalman.time_update();
        kalman.measurement_update(z);
    }

    EXPECT_TRUE(test::allclose(kalman.x(), x, reltol, abstol)) << output_matrices(kalman.x().transpose(), x.transpose());
}

