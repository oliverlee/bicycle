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
        const model::Bicycle& bicycle, bool dt = true, bool v = true,
        bool M = true, bool C1 = true, bool K0 = true, bool K2 = true) {
    double v_ = 0;
    double dt_ = 0;
    ::fbs::SecondOrderMatrix* M_ = nullptr;
    ::fbs::SecondOrderMatrix* C1_ = nullptr;
    ::fbs::SecondOrderMatrix* K0_ = nullptr;
    ::fbs::SecondOrderMatrix* K2_ = nullptr;

    if (K2 == true) {
        auto M__ = second_order_matrix(bicycle.K2());
        M_ = &M__;
    }
    if (K0 == true) {
        auto K0__ = second_order_matrix(bicycle.K0());
        K0_ = &K0__;
    }
    if (C1 == true) {
        auto C1__ = second_order_matrix(bicycle.C1());
        C1_ = &C1__;
    }
    if (M == true) {
        auto M__ = second_order_matrix(bicycle.M());
        M_ = &M__;
    }
    if (dt == true) {
        dt_ = bicycle.dt();
    }
    if (v == true) {
        v_ = bicycle.v();
    }
    return CreateBicycle(fbb, v_, dt_, M_, C1_, K0_, K2_);
}

flatbuffers::Offset<::fbs::Kalman> create_kalman(
        flatbuffers::FlatBufferBuilder& fbb,
        const observer::Kalman<model::Bicycle>& kalman, bool x = true,
        bool P = true, bool Q = true, bool R = true, bool K = true) {
    ::fbs::State* x_ = nullptr;
    ::fbs::SymmetricStateMatrix* P_ = nullptr;
    ::fbs::SymmetricStateMatrix* Q_ = nullptr;
    ::fbs::SymmetricOutputMatrix* R_ = nullptr;
    ::fbs::KalmanGainMatrix* K_ = nullptr;

    if (K == true) {
        auto K__ = kalman_gain_matrix(kalman.K());
        K_ = &K__;
    }
    if (R == true) {
        auto R__ = symmetric_output_matrix(kalman.R());
        R_ = &R__;
    }
    if (Q == true) {
        auto Q__ = symmetric_state_matrix(kalman.Q());
        Q_ = &Q__;
    }
    if (P == true) {
        auto P__ = symmetric_state_matrix(kalman.P());
        P_ = &P__;
    }
    if (x == true) {
        auto x__ = state(kalman.x());
        x_ = &x__;
    }
    return CreateKalman(fbb, x_, P_, Q_, R_, K_);
}


flatbuffers::Offset<::fbs::Lqr> create_lqr(
        flatbuffers::FlatBufferBuilder& fbb,
        const controller::Lqr<model::Bicycle>& lqr, bool n = true,
        bool r = true, bool P = true, bool Q = true, bool R = true,
        bool K = true) {
    uint32_t n_ = 0;
    ::fbs::State* r_ = nullptr;
    ::fbs::SymmetricStateMatrix* P_ = nullptr;
    ::fbs::SymmetricStateMatrix* Q_ = nullptr;
    ::fbs::SymmetricInputMatrix* R_ = nullptr;
    ::fbs::LqrGainMatrix* K_ = nullptr;

    if (K == true) {
        auto K__ = lqr_gain_matrix(lqr.K());
        K_ = &K__;
    }
    if (P == true) {
        auto P__ = symmetric_state_matrix(lqr.P());
        P_ = &P__;
    }
    if (R == true) {
        auto R__ = symmetric_input_matrix(lqr.R());
        R_ = &R__;
    }
    if (Q == true) {
        auto Q__ = symmetric_state_matrix(lqr.Q());
        Q_ = &Q__;
    }
    if (r == true) {
        auto r__ = state(lqr.xt());
        r_ = &r__;
    }
    if (n == true) {
        n_ = lqr.horizon_iterations();
    }
    return CreateLqr(fbb, n_, r_, Q_, R_, P_, K_);
}

} // namespace fbs
