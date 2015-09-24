#pragma once
#include "bicycle.h"
#include "kalman.h"
#include "lqr.h"
#include "sample_generated.h"

namespace fbs {

// TODO: generate per DiscreteLinear derived class
::fbs::State state(const model::Bicycle::state_t& x) {
    return ::fbs::State(x(0), x(1), x(2), x(3));
}

::fbs::Input input(const model::Bicycle::input_t& u) {
    return ::fbs::Input(u(0), u(1));
}

::fbs::Output output(const model::Bicycle::output_t& y) {
    return ::fbs::Output(y(0), y(1));
}

::fbs::SymmetricStateMatrix symmetric_state_matrix(const model::Bicycle::state_matrix_t& m) {
    return ::fbs::SymmetricStateMatrix(m(0, 0), m(0, 1), m(0, 2), m(0, 3),
            m(1, 1), m(1, 2), m(1, 3), m(2, 2), m(2, 3), m(3, 3));
}

::fbs::SymmetricInputMatrix symmetric_input_matrix(const controller::Lqr<model::Bicycle>::input_cost_t& m) {
    return ::fbs::SymmetricInputMatrix(m(0, 0), m(0, 1), m(1, 1));
}

::fbs::SymmetricOutputMatrix symmetric_output_matrix(
        const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t& m) {
    return ::fbs::SymmetricOutputMatrix(m(0, 0), m(0, 1), m(1, 1));
}

::fbs::SecondOrderMatrix second_order_matrix(const model::Bicycle::second_order_matrix_t& m) {
    return ::fbs::SecondOrderMatrix(m(0, 0), m(0, 1), m(1, 0), m(1, 1));
}

::fbs::KalmanGainMatrix kalman_gain_matrix(const observer::Kalman<model::Bicycle>::kalman_gain_t& k) {
    return ::fbs::KalmanGainMatrix(k(0, 0), k(0, 1), k(0, 2), k(0, 3),
            k(1, 0), k(1, 1), k(1, 2), k(1, 3));
}

::fbs::LqrGainMatrix lqr_gain_matrix(const controller::Lqr<model::Bicycle>::lqr_gain_t& k) {
    return ::fbs::LqrGainMatrix(k(0, 0), k(0, 1), k(0, 2), k(0, 3),
            k(1, 0), k(1, 1), k(1, 2), k(1, 3));
}

} // namespace fbs
