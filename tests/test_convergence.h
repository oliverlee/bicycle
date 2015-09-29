#pragma once
#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "kalman.h"
#include "lqr.h"

class ConvergenceTest: public ::testing::TestWithParam<double> {
    public:
        virtual void SetUp();
        virtual void TearDown();
        virtual void simulate() = 0;
        void test_state_near(model::Bicycle::state_t actual, model::Bicycle::state_t expected);
        model::Bicycle::state_t x_true();

    protected:
        static const double m_roll_tol;
        static const double m_steer_tol;
        static const double m_roll_rate_tol;
        static const double m_steer_rate_tol;
        static const double m_dt;
        static const uint32_t m_N; // simulation length
        static const uint32_t m_n; // horizon length
        model::Bicycle* m_bicycle;
        observer::Kalman<model::Bicycle>* m_kalman;
        controller::Lqr<model::Bicycle>* m_lqr;
        model::Bicycle::state_t m_x;

        std::random_device m_rd; // used only to seed rng
        std::mt19937 m_gen;
        std::normal_distribution<> m_r0;
        std::normal_distribution<> m_r1;
};

inline model::Bicycle::state_t ConvergenceTest::x_true() {
    return m_x;
}
