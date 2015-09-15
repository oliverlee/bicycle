#pragma once
#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "kalman.h"
#include "lqr.h"
#include "parameters.h"

class ConvergenceTest: public ::testing::TestWithParam<double> {
    public:
        virtual void SetUp();
        virtual void TearDown();
        virtual void simulate() = 0;
        void test_state_near(model::Bicycle::state_t actual);

    protected:
        const double roll_tol = 2.5 * constants::as_radians;
        const double steer_tol = 0.1 * constants::as_degrees;
        const double roll_rate_tol = 10 * roll_tol;
        const double steer_rate_tol = steer_tol;
        const double dt = 1.0/200;
        const uint32_t N = 1000; // simulation length
        const uint32_t n = 100; // horizon length
        model::Bicycle* bicycle;
        observer::Kalman<model::Bicycle>* kalman;
        controller::Lqr<model::Bicycle>* lqr;
        model::Bicycle::state_t x;

        std::random_device rd; // used only to seed rng
        std::mt19937 gen;
        std::normal_distribution<> r0;
        std::normal_distribution<> r1;
};
