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

flatbuffers::Offset<::fbs::Bicycle> create_bicycle(
        flatbuffers::FlatBufferBuilder& fbb,
        const model::Bicycle& bicycle) {
    auto M = second_order_matrix(bicycle.M());
    auto C1 = second_order_matrix(bicycle.C1());
    auto K0 = second_order_matrix(bicycle.K0());
    auto K2 = second_order_matrix(bicycle.K2());
    return CreateBicycle(fbb, bicycle.v(), bicycle.dt(), &M, &C1, &K0, &K2);
}

flatbuffers::Offset<::fbs::Kalman> create_kalman(
        flatbuffers::FlatBufferBuilder& fbb,
        const observer::Kalman<model::Bicycle>& kalman) {
    auto x = state(kalman.x());
    auto P = symmetric_state_matrix(kalman.P());
    auto Q = symmetric_state_matrix(kalman.Q());
    auto R = symmetric_output_matrix(kalman.R());
    auto K = kalman_gain_matrix(kalman.K());
    return CreateKalman(fbb, &x, &P, &Q, &R, &K);
}

flatbuffers::Offset<::fbs::Kalman> create_kalman_no_state_input_cost(
        flatbuffers::FlatBufferBuilder& fbb,
        const observer::Kalman<model::Bicycle>& kalman) {
    auto x = state(kalman.x());
    auto P = symmetric_state_matrix(kalman.P());
    auto K = kalman_gain_matrix(kalman.K());
    return CreateKalman(fbb, &x, &P, nullptr, nullptr, &K);
}

flatbuffers::Offset<::fbs::Lqr> create_lqr(flatbuffers::FlatBufferBuilder& fbb,
        const controller::Lqr<model::Bicycle>& lqr) {
    auto r = state(lqr.xt());
    auto P = symmetric_state_matrix(lqr.P());
    auto Q = symmetric_state_matrix(lqr.Q());
    auto R = symmetric_input_matrix(lqr.R());
    auto K = lqr_gain_matrix(lqr.K());
    return CreateLqr(fbb, lqr.horizon_iterations(), &r, &Q, &R, &P, &K);
}

flatbuffers::Offset<::fbs::Lqr> create_lqr_no_horizon_state_input_cost(
        flatbuffers::FlatBufferBuilder& fbb,
        const controller::Lqr<model::Bicycle>& lqr) {
    auto r = state(lqr.xt());
    auto P = symmetric_state_matrix(lqr.P());
    auto K = lqr_gain_matrix(lqr.K());
    return CreateLqr(fbb, 0, &r, nullptr, nullptr, &P, &K);
}

} // namespace fbs
