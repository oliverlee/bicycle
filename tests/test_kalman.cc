#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"
#include "test_utilities.h"

// TODO: test estimation with model error
class KalmanConvergenceTest: public ::testing::TestWithParam<double> {
    public:
        virtual void SetUp() {
            bicycle = new model::Bicycle(parameters::benchmark::M, parameters::benchmark::C1,
                    parameters::benchmark::K0, parameters::benchmark::K2, GetParam(), dt);
            bicycle->set_C(parameters::defaultvalue::bicycle::C);
            kalman = new observer::Kalman<model::Bicycle>(*bicycle,
                    parameters::defaultvalue::kalman::Q,
                    parameters::defaultvalue::kalman::R,
                    model::Bicycle::state_t::Zero(),
                    parameters::defaultvalue::kalman::P0);
            gen = std::mt19937(rd());
            r0 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(0, 0));
            r1 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(1, 1));

            // set initial state
            x << 0, 10, 10, 0; // define x in degrees
            x *= constants::as_radians;
        }

        virtual void TearDown() {
            delete bicycle;
            delete kalman;
            bicycle = nullptr;
            kalman = nullptr;
        }

        void simulate() {
            for(unsigned int i = 0; i < N; ++i) {
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

        void test_estimate_state_near() {
            auto actual = kalman->x();
            auto expected = x;
            EXPECT_NEAR(actual(0), expected(0), roll_tol);
            EXPECT_NEAR(actual(1), expected(1), steer_tol);
            EXPECT_NEAR(actual(2), expected(2), roll_rate_tol);
            EXPECT_NEAR(actual(3), expected(3), steer_rate_tol);
        }

    protected:
        const double roll_tol = 1 * constants::as_radians;
        const double steer_tol = 0.1 * constants::as_degrees;
        const double roll_rate_tol = 10 * roll_tol * constants::as_radians;
        const double steer_rate_tol = steer_tol * constants::as_degrees;
        const double dt = 1.0/200;
        const uint32_t N = 1000; // simulation length
        model::Bicycle* bicycle;
        observer::Kalman<model::Bicycle>* kalman;
        model::Bicycle::state_t x;

        const observer::Kalman<model::Bicycle>::error_covariance_t P0 =
            parameters::defaultvalue::kalman::P0;
        const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q =
            parameters::defaultvalue::kalman::Q;
        const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R =
            parameters::defaultvalue::kalman::R;

        std::random_device rd; // used only to seed rng
        std::mt19937 gen;
        std::normal_distribution<> r0;
        std::normal_distribution<> r1;
};

TEST_P(KalmanConvergenceTest, ZeroInput) {
    simulate();
    test_estimate_state_near();
}

TEST_P(KalmanConvergenceTest, RandomInput) {
    simulate_with_random_steer_input();
    test_estimate_state_near();
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    KalmanConvergenceTest,
    ::testing::Range(0.5, 9.5, 0.5));
