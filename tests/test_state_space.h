#pragma once
#include <random>
#include "gtest/gtest.h"
#include "bicycle.h"

class StateSpaceTest : public ::testing::Test {
    public:
        virtual void SetUp();
        virtual void TearDown();

    protected:
        model::Bicycle* bicycle;
};

inline void StateSpaceTest::SetUp() {
    bicycle = new model::Bicycle(
            parameters::benchmark::M,
            parameters::benchmark::C1,
            parameters::benchmark::K0,
            parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            parameters::benchmark::rear_wheel_radius,
            parameters::benchmark::front_wheel_radius,
            0.0, 0.0);
}

inline void StateSpaceTest::TearDown() {
    delete bicycle;
    bicycle = nullptr;
}
