#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: Add convergence with model error
class LqrConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for(unsigned int i = 0; i < N; ++i) {
                auto u = lqr->control_calculate(x);
                x = bicycle->x_next(x, u);
            }
        }
};

TEST_P(LqrConvergenceTest, ZeroReference) {
    simulate();
    test_state_near(x_true(), model::Bicycle::state_t::Zero());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    LqrConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
