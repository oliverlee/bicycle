#include "test_convergence.h"

void ConvergenceTest::SetUp() {
    bicycle = new model::Bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, GetParam(), dt);
    bicycle->set_C(parameters::defaultvalue::bicycle::C);
    kalman = new observer::Kalman<model::Bicycle>(*bicycle,
            parameters::defaultvalue::kalman::Q,
            parameters::defaultvalue::kalman::R,
            model::Bicycle::state_t::Zero(),
            parameters::defaultvalue::kalman::P0);
    lqr = new controller::Lqr<model::Bicycle>(*bicycle,
            controller::Lqr<model::Bicycle>::state_cost_t::Identity(),
            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity(),
            model::Bicycle::state_t::Zero(), n);
    gen = std::mt19937(rd());
    r0 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(0, 0));
    r1 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(1, 1));

    // set initial state
    x << 0, 10, 10, 0; // define x in degrees
    x *= constants::as_radians;
}

void ConvergenceTest::TearDown() {
    delete bicycle;
    delete kalman;
    delete lqr;
    bicycle = nullptr;
    kalman = nullptr;
    lqr = nullptr;
}

void ConvergenceTest::test_state_near(model::Bicycle::state_t actual, model::Bicycle::state_t expected) {
    EXPECT_NEAR(actual(0), expected(0), roll_tol);
    EXPECT_NEAR(actual(1), expected(1), steer_tol);
    EXPECT_NEAR(actual(2), expected(2), roll_rate_tol);
    EXPECT_NEAR(actual(3), expected(3), steer_rate_tol);
}
