#include <array>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "bicycle.h"
#include "constants.h"

// TODO: Investigate matrix rank threshold.
// Current threshold only results in capsize speed as rank deficient, not weave speed.
namespace {
    const double rank_threshold = 1e-5;
} // namespace

namespace model {

Bicycle::Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        double v, double dt, const state_space_map_t* discrete_state_space_map) :
    m_M(M), m_C1(C1), m_K0(K0), m_K2(K2),
    m_expAT(m_AT), m_discrete_state_space_map(discrete_state_space_map) {
    initialize_state_space_matrices();

    // set forward speed, sampling time and update state matrices
    set_v(v, dt);
}

Bicycle::Bicycle(const char* param_file, double v, double dt,
        const state_space_map_t* discrete_state_space_map) :
    m_expAT(m_AT), m_discrete_state_space_map(discrete_state_space_map) {
    // set M, C1, K0, K2 matrices from file
    set_matrices_from_file(param_file);

    initialize_state_space_matrices();

    // set forward speed, sampling time and update state matrices
    set_v(v, dt);
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
                (void)t;
                dxdt.block<n, 1>(0, 0) = m_A*xu.block<n, 1>(0, 0) + m_B*xu.block<m, 1>(n, 0);
            }, xu, 0.0, xout, dt);
    return xout.block<n, 1>(0, 0);
}

Bicycle::state_t Bicycle::x_integrate(const Bicycle::state_t& x, double dt) const {
    state_t xout;

    m_stepper_noinput.do_step([this](const state_t& x, state_t& dxdt, const double t) -> void {
                (void)t;
                dxdt = m_A*x;
            }, x, 0.0, xout, dt);
    return xout;
}

void Bicycle::set_v(double v, double dt) {
    /* system state space is parameterized by forward speed v
     * this function sets forward speed and calculates the state space matrices
     * and additionally calculates discrete time state space if sampling time is nonzero
     */
    m_v = v;
    m_dt = dt;

    // M is positive definite so use the Cholskey decomposition in solving the linear system
    m_A.bottomLeftCorner<o, o>() = -m_M.llt().solve(constants::g*m_K0 + m_v*m_v*m_K2);
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
            A_qr.setThreshold(rank_threshold);
            if (A_qr.rank() < n) {
                std::cout << "Warning: A is (near) singular and a precomputed Bd has not been " <<
                    "provided in the discrete state space map.\nComputation of Bd may be inaccurate.\n";
            }
            m_Bd = A_qr.solve((m_Ad - state_matrix_t::Identity())*m_B);
        }
    }
}

bool Bicycle::discrete_state_space_lookup(const state_space_map_key_t& k) {
    if (m_discrete_state_space_map == nullptr) {
        return false;
    }
    auto search = m_discrete_state_space_map->find(k);
    if (search == m_discrete_state_space_map->end()) {
        return false;
    }

    // discrete state space matrices Ad, Bd have been provided for speed v, sample time dt.
    m_Ad = search->second.first;
    m_Bd = search->second.second;
    return true;
}

void Bicycle::set_matrices_from_file(const char* param_file) {
    const unsigned int num_elem = o*o;
    std::array<double, 4*num_elem> buffer;

    std::fstream pf(param_file, std::ios_base::in);
    if (!pf.good()) {
        throw std::invalid_argument("Invalid matrix parameter file provided.");
    }

    for (auto& d: buffer) {
        pf >> d;
    }
    pf.close();

    m_M = Eigen::Map<second_order_matrix_t>(buffer.data()).transpose();
    m_C1 = Eigen::Map<second_order_matrix_t>(buffer.data() + num_elem).transpose();
    m_K0 = Eigen::Map<second_order_matrix_t>(buffer.data() + 2*num_elem).transpose();
    m_K2 = Eigen::Map<second_order_matrix_t>(buffer.data() + 3*num_elem).transpose();
}

void Bicycle::initialize_state_space_matrices() {
    m_A.setZero();
    m_B.setZero();
    m_C.setZero();
    m_D.setZero();
    m_Ad.setZero();
    m_Bd.setZero();

    // matrix used for calculating discrete time state space matrices
    // a reference of this matrix must be provided to the MatrixExponential class
    m_AT.setZero();

    // set constant parts of state and input matrices
    m_A.topRightCorner<o, o>().setIdentity();
    m_B.bottomLeftCorner<o, o>() = m_M.inverse();
}

} // namespace model
