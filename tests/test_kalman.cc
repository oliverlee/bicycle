#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: test estimation with model error
class KalmanConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for (unsigned int i = 0; i < m_N; ++i) {
                m_x = m_bicycle->x_next(m_x);

                auto z = m_bicycle->y(m_x);
                z(0) += m_r0(m_gen);
                z(1) += m_r1(m_gen);

                m_kalman->time_update();
                m_kalman->measurement_update(z);
            }
        }

        void simulate_with_random_steer_input() {
            // observed steer torques in experiments are between 1 to 3 Nm
            std::normal_distribution<> ru = std::normal_distribution<>(0, 2); // Nm, steer torque
            model::Bicycle::input_t u = model::Bicycle::input_t::Zero();

            for (unsigned int i = 0; i < m_N; ++i) {
                u(1) = ru(m_gen);
                m_x = m_bicycle->x_next(m_x, u);

                auto z = m_bicycle->y(m_x, u);
                z(0) += m_r0(m_gen);
                z(1) += m_r1(m_gen);

                m_kalman->time_update(u);
                m_kalman->measurement_update(z);
            }
        }
};

TEST_P(KalmanConvergenceTest, ZeroInput) {
    simulate();
    test_state_near(m_kalman->x(), x_true());
}

TEST_P(KalmanConvergenceTest, RandomInput) {
    simulate_with_random_steer_input();
    test_state_near(m_kalman->x(), x_true());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    KalmanConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
