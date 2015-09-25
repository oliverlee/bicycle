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

::fbs::StateMatrix state_matrix(const model::Bicycle::state_matrix_t& a) {
    return ::fbs::StateMatrix(
            a(0, 0), a(0, 1), a(0, 2), a(0, 3),
            a(1, 0), a(1, 1), a(1, 2), a(1, 3),
            a(2, 0), a(2, 1), a(2, 2), a(2, 3),
            a(3, 0), a(3, 1), a(3, 2), a(3, 3));
}

::fbs::InputMatrix input_matrix(const model::Bicycle::input_matrix_t& b) {
    return ::fbs::InputMatrix(
            b(0, 0), b(0, 1),
            b(1, 0), b(1, 1),
            b(2, 0), b(2, 1),
            b(3, 0), b(3, 1));
}

::fbs::OutputMatrix output_matrix(const model::Bicycle::output_matrix_t& c) {
    return ::fbs::OutputMatrix(
            c(0, 0), c(0, 1), c(0, 2), c(0, 3),
            c(1, 0), c(1, 1), c(1, 2), c(1, 3));
}

::fbs::FeedthroughMatrix feedthrough_matrix(const model::Bicycle::feedthrough_matrix_t& d) {
    return ::fbs::FeedthroughMatrix(
            d(0, 0), d(0, 1),
            d(1, 0), d(1, 1));
}

::fbs::SymmetricStateMatrix symmetric_state_matrix(const model::Bicycle::state_matrix_t& m) {
    return ::fbs::SymmetricStateMatrix(
            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
                     m(1, 1), m(1, 2), m(1, 3),
                              m(2, 2), m(2, 3),
                                       m(3, 3));
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
    return ::fbs::KalmanGainMatrix(
            k(0, 0), k(0, 1), k(0, 2), k(0, 3),
            k(1, 0), k(1, 1), k(1, 2), k(1, 3));
}

::fbs::LqrGainMatrix lqr_gain_matrix(const controller::Lqr<model::Bicycle>::lqr_gain_t& k) {
    return ::fbs::LqrGainMatrix(
            k(0, 0), k(0, 1),
            k(0, 2), k(0, 3),
            k(1, 0), k(1, 1),
            k(1, 2), k(1, 3));
}


flatbuffers::Offset<::fbs::Bicycle> create_bicycle(
        flatbuffers::FlatBufferBuilder& fbb,
        const model::Bicycle& bicycle, bool dt = true, bool v = true,
        bool M = true, bool C1 = true, bool K0 = true, bool K2 = true,
        bool Ad = true, bool Bd = true, bool Cd = true, bool Dd = true) {
    double v_ = 0;
    double dt_ = 0;
    ::fbs::SecondOrderMatrix* M_ = nullptr;
    ::fbs::SecondOrderMatrix* C1_ = nullptr;
    ::fbs::SecondOrderMatrix* K0_ = nullptr;
    ::fbs::SecondOrderMatrix* K2_ = nullptr;
    ::fbs::StateMatrix* Ad_ = nullptr;
    ::fbs::InputMatrix* Bd_ = nullptr;
    ::fbs::OutputMatrix* Cd_ = nullptr;
    ::fbs::FeedthroughMatrix* Dd_ = nullptr;

    if (v == true) {
        v_ = bicycle.v();
    }
    if (dt == true) {
        dt_ = bicycle.dt();
    }
    if (M == true) {
        auto M__ = second_order_matrix(bicycle.M());
        M_ = &M__;
    }
    if (C1 == true) {
        auto C1__ = second_order_matrix(bicycle.C1());
        C1_ = &C1__;
    }
    if (K0 == true) {
        auto K0__ = second_order_matrix(bicycle.K0());
        K0_ = &K0__;
    }
    if (K2 == true) {
        auto K2__ = second_order_matrix(bicycle.K2());
        K2_ = &K2__;
    }
    if (Ad == true) {
        auto Ad__ = state_matrix(bicycle.Ad());
        Ad_ = &Ad__;
    }
    if (Bd == true) {
        auto Bd__ = input_matrix(bicycle.Bd());
        Bd_ = &Bd__;
    }
    if (Cd == true) {
        auto Cd__ = output_matrix(bicycle.Cd());
        Cd_ = &Cd__;
    }
    if (Dd == true) {
        auto Dd__ = feedthrough_matrix(bicycle.Dd());
        Dd_ = &Dd__;
    }
    return CreateBicycle(fbb, v_, dt_, M_, C1_, K0_, K2_, Ad_, Bd_, Cd_, Dd_);
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
