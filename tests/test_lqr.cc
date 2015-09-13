#include "gtest/gtest.h"
#include "bicycle.h"
#include "lqr.h"
#include "parameters.h"
#include "test_utilities.h"

// TODO: Add more extensive testing
namespace {
    const double reltol = 0; // target is zero
    const double abstol = 1e-3;
    const double dt = 1.0/200;
    const double v = 4.0;
    const uint32_t N = 1000; // simulation length
    const size_t n = 100;  // length of horizon in samples

    model::Bicycle::state_t x( // initial state
            (model::Bicycle::state_t() <<
                0, 10, 10, 0).finished() * constants::as_radians);

    const model::Bicycle::state_t xt( // target state
            model::Bicycle::state_t::Zero());

    const controller::Lqr<model::Bicycle>::state_cost_t Q(
            controller::Lqr<model::Bicycle>::state_cost_t::Identity());

    const controller::Lqr<model::Bicycle>::input_cost_t R(
            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity());
} // namespace

TEST(Lqr, Convergence) {
    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, v, dt);
    controller::Lqr<model::Bicycle> lqr(bicycle, Q, R, xt, n);

    for(unsigned int i = 0; i < N; ++i) {
        auto u = lqr.control_calculate(x);
        x = bicycle.x_next(x, u);
    }

    EXPECT_TRUE(test::allclose(x, xt, reltol, abstol)) << output_matrices(x.transpose(), xt.transpose());
}
