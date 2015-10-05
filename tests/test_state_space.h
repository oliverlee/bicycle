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
    bicycle = new model::Bicycle(0.0, 0.0);
}

inline void StateSpaceTest::TearDown() {
    delete bicycle;
    bicycle = nullptr;
}
