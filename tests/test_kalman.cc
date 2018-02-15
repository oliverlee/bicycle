#include "gtest/gtest.h"
#include "test_convergence.h"
#include <boost/math/constants/constants.hpp>

// TODO: test estimation with model error
class KalmanConvergenceTest: public ConvergenceTest {
    public:
        void simulate(size_t N = default_simulation_length) {
            for (unsigned int i = 0; i < N; ++i) {
                m_x = m_bicycle->update_state(m_x);

                auto z = m_bicycle->calculate_output(m_x);
                z(0) += m_r0(m_gen);
                z(1) += m_r1(m_gen);

                m_kalman->time_update();
                m_kalman->measurement_update(z);

                // stop simulation if frame is horizontal (bike has fallen)
                if ((m_x[1] >= boost::math::constants::half_pi<model::real_t>()) ||
                        (m_x[1] < -boost::math::constants::half_pi<model::real_t>())) {
                    break;
                }
            }
        }

        void simulate_with_random_steer_input(size_t N = default_simulation_length) {
            // observed steer torques in experiments are between 1 to 3 Nm
            std::normal_distribution<> ru = std::normal_distribution<>(0, 2); // Nm, steer torque
            model::BicycleWhipple::input_t u = model::BicycleWhipple::input_t::Zero();

            for (unsigned int i = 0; i < N; ++i) {
                u(1) = ru(m_gen);
                m_x = m_bicycle->update_state(m_x, u);

                auto z = m_bicycle->calculate_output(m_x, u);
                z(0) += m_r0(m_gen);
                z(1) += m_r1(m_gen);

                m_kalman->time_update(u);
                m_kalman->measurement_update(z);

                // stop simulation if frame is horizontal (bike has fallen)
                if ((m_x[1] >= boost::math::constants::half_pi<model::real_t>()) ||
                        (m_x[1] < -boost::math::constants::half_pi<model::real_t>())) {
                    break;
                }
            }
        }
};

class KalmanConvergenceTest_HighSpeed: public KalmanConvergenceTest { };
class KalmanConvergenceTest_LowSpeed: public KalmanConvergenceTest { };

TEST_P(KalmanConvergenceTest_HighSpeed, ZeroInput) {
    simulate();
    test_state_near(m_kalman->x(), x_true());
}

TEST_P(KalmanConvergenceTest_HighSpeed, RandomInput) {
    simulate_with_random_steer_input();
    test_state_near(m_kalman->x(), x_true());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_3_10,
    KalmanConvergenceTest_HighSpeed,
    ::testing::Range(static_cast<model::real_t>(3.0),
        static_cast<model::real_t>(10.0),
        static_cast<model::real_t>(0.5)));

TEST_P(KalmanConvergenceTest_LowSpeed, ZeroInput) {
    simulate_with_random_steer_input(200); // 100 iterations instead of 1000
    test_state_near(m_kalman->x(), x_true());
}

TEST_P(KalmanConvergenceTest_LowSpeed, RandomInput) {
    simulate_with_random_steer_input(200); // 100 iterations instead of 1000
    test_state_near(m_kalman->x(), x_true());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_3,
    KalmanConvergenceTest_LowSpeed,
    ::testing::Range(static_cast<model::real_t>(1.0),
        static_cast<model::real_t>(3.0),
        static_cast<model::real_t>(0.5)));
