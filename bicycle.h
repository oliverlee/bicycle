#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>


namespace model {

/* State and Input Definitions
 * state: [roll angle, steer angle, roll rate, steer rate]
 * input: [roll torque, steer torque]
 * output: [steer angle]
 */

class Bicycle{
    public:
        static constexpr unsigned int n = 4; // state size
        static constexpr unsigned int m = 2; // input size
        static constexpr unsigned int l = 1; // output size, only steer angle is measured
        static constexpr unsigned int o = n/2; // second order state size

        using state_t = Eigen::Matrix<double, n, 1>;
        using input_t = Eigen::Matrix<double, m, 1>;
        using output_t = Eigen::Matrix<double, l, 1>;
        using state_matrix_t = Eigen::Matrix<double, n, n>;
        using input_matrix_t = Eigen::Matrix<double, n, m>;
        using output_matrix_t = Eigen::Matrix<double, l, n>;
        using feedthrough_matrix_t = Eigen::Matrix<double, l, m>;
        using second_order_matrix_t = Eigen::Matrix<double, o, o>;

        Bicycle(const char* paramfile, double v, double dt = 0.0);
        void set_matrices_from_file(const char* param_file);

        state_t x_next(const state_t& x, const input_t& u) const;
        state_t x_integrate(const state_t& x, const input_t& u, double dt) const;
        output_t y(const state_t& x, const input_t& u) const;
        state_t x_next(const state_t& x) const;
        state_t x_integrate(const state_t& x, double dt) const;
        output_t y(const state_t& x) const;
        void set_v(double v, double dt = 0.0);

        // (pseudo) parameter accessors
        state_matrix_t A() const {
            return m_A;
        }
        input_matrix_t B() const {
            return m_B;
        }
        output_matrix_t C() const {
            return m_C;
        }
        feedthrough_matrix_t D() const {
            return m_D;
        }
        state_matrix_t Ad() const {
            return m_Ad;
        }
        input_matrix_t Bd() const {
            return m_Bd;
        }
        output_matrix_t Cd() const {
            return m_C;
        }
        feedthrough_matrix_t Dd() const {
            return m_D;
        }
        second_order_matrix_t M() const {
            return m_M;
        }
        second_order_matrix_t C1() const {
            return m_C1;
        }
        second_order_matrix_t K0() const {
            return m_K0;
        }
        second_order_matrix_t K2() const {
            return m_K2;
        }
        double v() const {
            return m_v;
        }
        double dt() const {
            return m_dt;
        }

    private:
        double m_v; // parameterized forward speed
        double m_dt; // sampling time of discrete time system
        second_order_matrix_t m_M;
        second_order_matrix_t m_C1;
        second_order_matrix_t m_K0;
        second_order_matrix_t m_K2;

        state_matrix_t m_A;
        input_matrix_t m_B;
        output_matrix_t m_C;
        feedthrough_matrix_t m_D;

        state_matrix_t m_Ad;
        input_matrix_t m_Bd;

        state_matrix_t m_AT;
        Eigen::MatrixExponential<state_matrix_t> m_expAT;

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
}; // class Bicycle

} // namespace model
