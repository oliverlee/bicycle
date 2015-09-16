#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: Add more extensive testing
class LqrKalmanConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for(unsigned int i = 0; i < N; ++i) {
                auto u = lqr->control_calculate(x);
                x = bicycle->x_next(x, u);

                auto z = bicycle->y(x);
                z(0) += r0(gen);
                z(1) += r1(gen);

                kalman->time_update(u);
                kalman->measurement_update(z);
            }
        }
};

TEST_P(LqrKalmanConvergenceTest, ZeroReference) {
    simulate();
    test_state_near(kalman->x(), x_true());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    LqrKalmanConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
