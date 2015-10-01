#pragma once
#include <unordered_map>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/functional/hash.hpp>
#include "discrete_linear.h"

namespace model {

/* State and Input Definitions
 * state: [roll angle, steer angle, roll rate, steer rate]
 * input: [roll torque, steer torque]
 * output: [*, *] - 2 outputs are defined, but must be specified by
 *                  setting the C and D matrices
 */

class Bicycle : public DiscreteLinear<4, 2, 2, 2> {
    public:
        /* We normally treat speed v as a double. However to allow for constant
         * time lookup along with quickly finding a key 'near' the requested
         * one, we convert speeds to a fixed precision integer.
         *
         * For example, if six digits after the decimal defines the precision used:
         * v = 6.024262 -> 6024262
         *
         * And the same is done with sample time dt and microsecond precision is used.
         * dt = 0.005 -> 5000
         *
         * Keys are defined as a pair:
         * (round(m_dt_key_precision*dt), round(m_v_key_precision*v)).
         * Where precision is set with m_dt_key_precision and m_v_key_precision.
         */
        using state_space_map_key_t = const std::pair<uint32_t, int32_t>;
        using state_space_map_value_t = const std::pair<state_matrix_t, input_matrix_t>;
        using state_space_map_t = std::unordered_map<state_space_map_key_t,
              state_space_map_value_t, boost::hash<state_space_map_key_t>>;

        Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                double wheelbase, double trail, double steer_axis_tilt,
                double v, double dt,
                const state_space_map_t* discrete_state_space_map = nullptr);
        Bicycle(const char* param_file, double v, double dt,
                const state_space_map_t* discrete_state_space_map = nullptr);

        virtual state_t x_next(const state_t& x, const input_t& u) const;
        state_t x_integrate(const state_t& x, const input_t& u, double dt) const;
        virtual output_t y(const state_t& x, const input_t& u) const;
        virtual state_t x_next(const state_t& x) const;
        state_t x_integrate(const state_t& x, double dt) const;
        virtual output_t y(const state_t& x) const;
        void set_v(double v, double dt);
        void set_C(const output_matrix_t& C);
        void set_D(const feedthrough_matrix_t& D);

        static constexpr state_space_map_key_t make_state_space_map_key(double v, double dt);
        bool discrete_state_space_lookup(const state_space_map_key_t& k);

        // (pseudo) parameter accessors
        state_matrix_t A() const;
        input_matrix_t B() const;
        output_matrix_t C() const;
        feedthrough_matrix_t D() const;
        virtual state_matrix_t Ad() const;
        virtual input_matrix_t Bd() const;
        virtual output_matrix_t Cd() const;
        virtual feedthrough_matrix_t Dd() const;
        second_order_matrix_t M() const;
        second_order_matrix_t C1() const;
        second_order_matrix_t K0() const;
        second_order_matrix_t K2() const;
        double wheelbase() const;
        double trail() const;
        double steer_axis_tilt() const;
        double v() const;
        virtual double dt() const;

    private:
        double m_v; // parameterized forward speed
        double m_dt; // sampling time of discrete time system
        second_order_matrix_t m_M;
        second_order_matrix_t m_C1;
        second_order_matrix_t m_K0;
        second_order_matrix_t m_K2;
        double m_w;
        double m_c;
        double m_lambda;

        state_matrix_t m_A;
        //input_matrix_t m_B; Use Cholesky decomposition of M
        Eigen::LLT<second_order_matrix_t> m_M_llt;
        output_matrix_t m_C;
        feedthrough_matrix_t m_D;

        state_matrix_t m_Ad;
        input_matrix_t m_Bd;

        state_matrix_t m_AT;
        Eigen::MatrixExponential<state_matrix_t> m_expAT;

        static constexpr uint32_t m_dt_key_precision = 1000;
        static constexpr int32_t m_v_key_precision = 1000000;

        state_space_map_t const* m_discrete_state_space_map;

        /* Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we can mark them as mutable.
         */
        using odeint_state_t = Eigen::Matrix<double, n + m, 1>;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            odeint_state_t, double, odeint_state_t, double,
            boost::numeric::odeint::vector_space_algebra> m_stepper;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            state_t, double, state_t, double,
            boost::numeric::odeint::vector_space_algebra> m_stepper_noinput;

        void set_parameters_from_file(const char* param_file);
        void initialize_state_space_matrices();
}; // class Bicycle

// define simple member functions using inline
inline void Bicycle::set_C(const output_matrix_t& C) {
    m_C = C;
}
inline void Bicycle::set_D(const feedthrough_matrix_t& D) {
    m_D = D;
}
inline constexpr Bicycle::state_space_map_key_t Bicycle::make_state_space_map_key(double v, double dt) {
    return state_space_map_key_t(m_dt_key_precision*dt, m_v_key_precision*v);
}
inline Bicycle::state_matrix_t Bicycle::A() const {
    return m_A;
}
inline Bicycle::input_matrix_t Bicycle::B() const {
    // Calculate M^-1 as we have explicitly asked for B
    input_matrix_t B;
    B.topRows<o>() = second_order_matrix_t::Zero();
    if (o < 5) {
        B.bottomRows<o>() = m_M.inverse();
    } else {
        B.bottomRows<o>() = m_M_llt.solve(second_order_matrix_t::Identity());
    }
    return B;
}
inline Bicycle::output_matrix_t Bicycle::C() const {
    return m_C;
}
inline Bicycle::feedthrough_matrix_t Bicycle::D() const {
    return m_D;
}
inline Bicycle::state_matrix_t Bicycle::Ad() const {
    return m_Ad;
}
inline Bicycle::input_matrix_t Bicycle::Bd() const {
    return m_Bd;
}
inline Bicycle::output_matrix_t Bicycle::Cd() const {
    return m_C;
}
inline Bicycle::feedthrough_matrix_t Bicycle::Dd() const {
    return m_D;
}
inline Bicycle::second_order_matrix_t Bicycle::M() const {
    return m_M;
}
inline Bicycle::second_order_matrix_t Bicycle::C1() const {
    return m_C1;
}
inline Bicycle::second_order_matrix_t Bicycle::K0() const {
    return m_K0;
}
inline Bicycle::second_order_matrix_t Bicycle::K2() const {
    return m_K2;
}
inline double Bicycle::wheelbase() const {
    return m_w;
}
inline double Bicycle::trail() const {
    return m_c;
}
inline double Bicycle::steer_axis_tilt() const {
    return m_lambda;
}
inline double Bicycle::v() const {
    return m_v;
}
inline double Bicycle::dt() const {
    return m_dt;
}

} // namespace model
