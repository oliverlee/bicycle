#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>
#include "bicycle.h"
#include "constants.h"

namespace {
    const double discretization_precision = Eigen::NumTraits<double>::dummy_precision();
} // namespace

namespace model {
Bicycle::Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        double wheelbase, double trail, double steer_axis_tilt,
        double rear_wheel_radius, double front_wheel_radius,
        double v, double dt, const state_space_map_t* discrete_state_space_map) :
    m_M(M), m_C1(C1), m_K0(K0), m_K2(K2),
    m_w(wheelbase), m_c(trail), m_lambda(steer_axis_tilt),
    m_rR(rear_wheel_radius), m_rF(front_wheel_radius),
    m_discrete_state_space_map(discrete_state_space_map) {
    initialize_state_space_matrices();

    // set forward speed, sampling time and update state matrices
    set_v(v, dt);
}

Bicycle::Bicycle(const char* param_file, double v, double dt,
        const state_space_map_t* discrete_state_space_map) :
    m_discrete_state_space_map(discrete_state_space_map) {
    // set M, C1, K0, K2 matrices from file
    set_parameters_from_file(param_file);
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
                dxdt.head<n>() = m_A*xu.head<n>();
                // Normally we would write dxdt = A*x + B*u but B is not stored
                // explicitly as that would require the calculation of
                // M.inverse(). As B = [   0  ], the product Bu = [      0   ]
                //                     [ M^-1 ]                   [ M^-1 * u ]
                dxdt.segment<o>(o) += m_M_llt.solve(xu.segment<o>(n));
            }, xu, 0.0, xout, dt);
    return xout.head<n>();
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

    // M is positive definite so use the Cholesky decomposition in solving the linear system
    m_A(0, 2) = v * std::cos(m_lambda) / m_w;
    m_A.block<o, o>(3, 1) = -m_M_llt.solve(constants::g*m_K0 + m_v*m_v*m_K2);
    m_A.bottomRightCorner<o, o>() = -m_M_llt.solve(m_v*m_C1);

    // Calculate M^-1 as we need it for discretization
    if (o < 5) { // inverse calculation is okay if matrix size is small
        m_B.bottomRows<o>() = m_M.inverse();
    } else {
        m_B.bottomRows<o>() = m_M_llt.solve(second_order_matrix_t::Identity());
    }

    if (m_dt == 0.0) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        state_space_map_key_t k = make_state_space_map_key(v, dt);
        if (!discrete_state_space_lookup(k)) {
            discretization_matrix_t AT = discretization_matrix_t::Zero();
            AT.topLeftCorner<n, n>() = m_A;
            AT.topRightCorner<n, m>() = m_B;
            AT *= dt;
            discretization_matrix_t T = AT.exp();
            if (!T.bottomLeftCorner<m, n>().isZero(discretization_precision) ||
                !T.bottomRightCorner<m, m>().isIdentity(discretization_precision)) {
                std::cout << "Warning: Discretization validation failed with v = " << v <<
                    ", dt = " << dt << ". Computation of Ad and Bd may be inaccurate.\n";
            }
            m_Ad = T.topLeftCorner<n, n>();
            m_Bd = T.topRightCorner<n, m>();
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

void Bicycle::set_parameters_from_file(const char* param_file) {
    const unsigned int num_elem = o*o;
    std::array<double, 4*num_elem + 5> buffer;

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
    m_w = buffer[4*num_elem];
    m_c = buffer[4*num_elem + 1];
    m_lambda = buffer[4*num_elem + 2];
    m_rR = buffer[4*num_elem + 3];
    m_rF = buffer[4*num_elem + 4];
}

void Bicycle::initialize_state_space_matrices() {
    m_A.setZero();
    m_B.setZero();
    m_C.setZero();
    m_D.setZero();
    m_Ad.setZero();
    m_Bd.setZero();

    // set constant parts of state and input matrices
    m_A(0, 4) = m_c * std::cos(m_lambda) / m_w;
    m_A.block<o, o>(1, 3).setIdentity();
    m_B.topRows<o>() = second_order_matrix_t::Zero();

    // We can write B in block matrix form as:
    // B.transpose() = [0 |  M.inverse().transpose()]
    // As M is positive definite, the Cholesky decomposition of M is stored and
    // used when necessary
    // m_B.bottomLeftCorner<o, o>() = m_M.inverse();
    m_M_llt.compute(m_M);
}

} // namespace model
