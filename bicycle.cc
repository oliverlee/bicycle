#include <array>
#include <fstream>
#include <iostream>
#include "bicycle.h"

namespace model {

const double g = 9.80665; // gravitational constant [m/s^2]

Bicycle::Bicycle(const char* param_file, double v, double dt,
        state_space_map_t const* discrete_state_space_map) :
    m_expAT(m_AT), m_discrete_state_space_map(discrete_state_space_map) {
    // set M, C1, K0, K2 matrices from file
    set_matrices_from_file(param_file);

    // initialize state space matrices to zero
    m_A.setZero();
    m_B.setZero();
    m_C.setZero();
    m_D.setZero();
    m_Ad.setZero();
    m_Bd.setZero();

    // matrix used for calculating discrete time state space matrices
    // a reference of this matrix must be provided to the MatrixExponential class
    m_AT.setZero();

    // set top right block to identity
    m_A.topRightCorner<o, o>().setIdentity();
    // set lower half of input matrix
    m_B.bottomLeftCorner<o, o>() = m_M.inverse();
    // set forward speed and (optionally sampling time) and update state matrix
    set_v(v, dt);
}

void Bicycle::set_matrices_from_file(const char* param_file) {
    const unsigned int num_elem = o*o;
    std::array<double, 4*num_elem> buffer;

    std::fstream pf(param_file, std::ios_base::in);
    for (auto& d: buffer) {
        pf >> d;
    }
    pf.close();

    m_M = Eigen::Map<second_order_matrix_t>(buffer.data()).transpose();
    m_C1 = Eigen::Map<second_order_matrix_t>(buffer.data() + num_elem).transpose();
    m_K0 = Eigen::Map<second_order_matrix_t>(buffer.data() + 2*num_elem).transpose();
    m_K2 = Eigen::Map<second_order_matrix_t>(buffer.data() + 3*num_elem).transpose();
}

Bicycle::state_t Bicycle::x_next(const Bicycle::state_t& x, const Bicycle::input_t& u) const {
    return m_Ad*x + m_Bd*u;
}

Bicycle::output_t Bicycle::y(const Bicycle::state_t& x, const Bicycle::input_t& u) const {
    return m_C*x + m_D*u;
}

Bicycle::state_t Bicycle::x_next(const Bicycle::state_t& x) const {
    return m_Ad*x;
}

Bicycle::output_t Bicycle::y(const Bicycle::state_t& x) const {
    return m_C*x;
}

Bicycle::state_t Bicycle::x_integrate(const Bicycle::state_t& x, const Bicycle::input_t& u, double dt) const {
    odeint_state_t xu;
    odeint_state_t xout;

    xu << x, u;
    m_stepper.do_step([this](const odeint_state_t& xu, odeint_state_t& dxdt, const double t) -> void {
                dxdt.block<n, 1>(0, 0) = m_A*xu.block<n, 1>(0, 0) + m_B*xu.block<m, 1>(n, 0);
            }, xu, 0.0, xout, dt);
    return xout.block<n, 1>(0, 0);
}

Bicycle::state_t Bicycle::x_integrate(const Bicycle::state_t& x, double dt) const {
    state_t xout;

    m_stepper_noinput.do_step([this](const state_t& x, state_t& dxdt, const double t) -> void {
                dxdt = m_A*x;
            }, x, 0.0, xout, dt);
    return xout;
}

void Bicycle::set_v(double v, double dt) {
    /* system state space is parameterized by forward speed v
     * this function sets forward speed and calculates the state space matrices
     * additionally, calculates discrete time state space if sampling time is nonzero
     */
    m_v = v;
    m_dt = dt;

    // M is positive definite so use the Cholskey decomposition in solving the linear system
    m_A.bottomLeftCorner<o, o>() = -m_M.llt().solve(g*m_K0 + m_v*m_v*m_K2);
    m_A.bottomRightCorner<o, o>() = -m_M.llt().solve(m_v*m_C1);

    if (m_dt == 0.0) { // discrete time state does not change
        m_AT.setZero();
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        m_AT = m_A*dt;
        state_space_map_key_t k = make_state_space_map_key(v, dt);
        if (!discrete_state_space_lookup(k)) {
            m_expAT.compute(m_Ad);
            Eigen::FullPivHouseholderQR<state_matrix_t> A_qr(m_A);
            // TODO: setThreshold() for rank deficiency instead of using default
            if (A_qr.rank() < n) {
                std::cout << "Warning: A is (near) singular and a precomputed Bd has not been " <<
                    "provided in the discrete state space map.\nComputation of Bd may be inaccurate.\n";
            }
            m_Bd = A_qr.solve((m_Ad - state_matrix_t::Identity())*m_B);
        }
    }
}

//inline constexpr Bicycle::state_space_map_key_t Bicycle::make_state_space_map_key(double v, double dt) {
//    return state_space_map_key_t(1000*dt, 10*v);
//}

bool Bicycle::discrete_state_space_lookup(const state_space_map_key_t& k) {
    if (m_discrete_state_space_map == nullptr) {
        return false;
    }
    auto search = m_discrete_state_space_map->find(k);
    if (search == m_discrete_state_space_map->end()) {
        return false;
    }

    // discrete state space matrices Ad, Bd have been provided for speed v.
    m_Ad = search->second.first;
    m_Bd = search->second.second;
    return true;
}

} // namespace model
