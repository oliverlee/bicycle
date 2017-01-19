#pragma once
#include <random>
#include "gtest/gtest.h"
#include "bicycle/whipple.h"

class StateSpaceTest : public ::testing::Test {
    public:
        virtual void SetUp();
        virtual void TearDown();

    protected:
        model::BicycleWhipple* bicycle;
};

inline void StateSpaceTest::SetUp() {
    bicycle = new model::BicycleWhipple(0.0, 0.0);
}

inline void StateSpaceTest::TearDown() {
    delete bicycle;
    bicycle = nullptr;
}
