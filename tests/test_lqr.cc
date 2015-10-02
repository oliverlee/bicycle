#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: Add convergence with model error
class LqrConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for(unsigned int i = 0; i < m_N; ++i) {
                auto u = m_lqr->control_calculate(m_x);
                m_x = m_bicycle->x_next(m_x, u);
            }
        }
};

TEST_P(LqrConvergenceTest, ZeroReference) {
    simulate();
    test_state_near(x_true(), bicycle_t::state_t::Zero(), 10.0);
}

TEST_P(LqrConvergenceTest, ReducedControlZeroReference) {
    m_lqr->set_R((lqr_t::input_cost_t() <<
            0, 0,
            0, 1).finished());
    simulate();
    test_state_near(x_true(), bicycle_t::state_t::Zero(), 10.0);
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    LqrConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
