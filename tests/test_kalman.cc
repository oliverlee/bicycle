#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: test estimation with model error
class KalmanConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for (unsigned int i = 0; i < N; ++i) {
                x = bicycle->x_next(x);

                auto z = bicycle->y(x);
                z(0) += r0(gen);
                z(1) += r1(gen);

                kalman->time_update();
                kalman->measurement_update(z);
            }
        }

        void simulate_with_random_steer_input() {
            // observed steer torques in experiments are between 1 to 3 Nm
            std::normal_distribution<> ru = std::normal_distribution<>(0, 2); // Nm, steer torque
            model::Bicycle::input_t u = model::Bicycle::input_t::Zero();

            for (unsigned int i = 0; i < N; ++i) {
                u(1) = ru(gen);
                x = bicycle->x_next(x, u);

                auto z = bicycle->y(x, u);
                z(0) += r0(gen);
                z(1) += r1(gen);

                kalman->time_update();
                kalman->measurement_update(z);
            }
        }
};

TEST_P(KalmanConvergenceTest, ZeroInput) {
    simulate();
    test_state_near(kalman->x());
}

TEST_P(KalmanConvergenceTest, RandomInput) {
    simulate_with_random_steer_input();
    test_state_near(kalman->x());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    KalmanConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
