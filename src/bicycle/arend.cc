#include <cmath>
#include "bicycle/arend.h"
#include "constants.h"

namespace {
    template <typename E>
    constexpr uint8_t index(E e) {
        return static_cast<uint8_t>(e);
    }
} // namespace

namespace model {
BicycleArend::BicycleArend(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt) {
    set_K();
}

BicycleArend::BicycleArend(const char* param_file, real_t v, real_t dt) :
    Bicycle(param_file, v, dt) {
    set_K();
}

BicycleArend::BicycleArend(real_t v, real_t dt) :
    Bicycle(v, dt) {
    set_K();
}

BicycleArend::state_t BicycleArend::update_state(const BicycleArend::state_t& x, const BicycleArend::input_t& u, const BicycleArend::measurement_t& z) const {
    full_state_t xout = integrate_full_state(make_full_state(auxiliary_state_t::Zero(), x), u, m_dt, z);
    return get_state_part(xout);
}

BicycleArend::full_state_t BicycleArend::integrate_full_state(const BicycleArend::full_state_t& xf, const BicycleArend::input_t& u, real_t t, const BicycleArend::measurement_t& z) const {
    /*
     * Simplified equations of motion are used to simulate the bicycle dynamics.
     * Roll/steer acceleration terms and input torque are ignored resulting in:
     *          (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
     *                          [delta] = [T_delta]
     *
     * Simplified equations of motion are used to calculate the handlebar feedback torque.
     * Roll/steer acceleration terms are ignored resulting in:
     *      [M_00 M_01] [phi_ddot  ] + v*C1 [phi_dot  ] + (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
     *      [M_10 M_11] [delta_ddot]        [delta_dot]                   [delta] = [T_delta]
     *
     * We assume T_phi is zero, and set M_01 = M_10 to zero.
     *      [M_00    0] [phi_ddot  ] + v*C1 [phi_dot  ] + (g*K0 + v^2*K2) [phi  ] = [      0]
     *      [   0 M_11] [delta_ddot]        [delta_dot]                   [delta] = [T_delta]
     *
     * From the roll equation, we have:
     *      M_00*phi_ddot + C_00*phi_dot + K_00*phi = -(M_01*delta_ddot + C_01*delta_dot + K_01*delta)
     *
     * and with simplifying zeros
     *      M_00*phi_ddot + C_00*phi_dot + K_00*phi = -(C_01*delta_dot + K_01*delta)
     *
     * which we can rewrite in ODE form as as
     *      [phi_dot ] = [phi_dot                                                      ]
     *      [phi_ddot] = [-(C_01*delta_dot + K_01*delta + C_00*phi_dot + K_00*phi)/M_00]
     *
     * Input u is not directly used.
     *
     * For more information, refer to: https://github.com/oliverlee/phobos/issues/161
     */
    (void)u;
    static constexpr auto x_index = index(full_state_index_t::x);
    static constexpr auto y_index = index(full_state_index_t::y);
    static constexpr auto rear_wheel_index = index(full_state_index_t::rear_wheel_angle);
    static constexpr auto pitch_index = index(full_state_index_t::pitch_angle);
    static constexpr auto yaw_index = index(full_state_index_t::yaw_angle);
    static constexpr auto roll_index = index(full_state_index_t::roll_angle);
    static constexpr auto steer_index = index(full_state_index_t::steer_angle);
    static constexpr auto roll_rate_index = index(full_state_index_t::roll_rate);
    static constexpr auto steer_rate_index = index(full_state_index_t::steer_rate);

    static constexpr auto yaw_index_A = index(state_index_t::yaw_angle);
    static constexpr auto steer_index_A = index(state_index_t::steer_angle);
    static constexpr auto steer_rate_index_A = index(state_index_t::steer_rate);

    const state_matrix_t&A = m_A;
    const real_t M_00 = m_M(0, 0);
    const real_t C_01 = m_v*m_C1(0, 1);
    const real_t K_01 = m_K(0, 1);
    const real_t C_00 = m_v*m_C1(0, 0);
    const real_t K_00 = m_K(0, 0);
    const real_t v = m_v;
    const real_t rr = m_rr;

    const real_t steer_angle_measurement = z[index(output_index_t::steer_angle)];
    const real_t steer_rate_measurement = z[index(output_index_t::steer_rate)];

    full_state_t xout = xf;

    m_stepper.do_step([&A, M_00, C_01, K_01, C_00, K_00, v, rr,
                       steer_angle_measurement, steer_rate_measurement](
                const full_state_t& x, full_state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent;

            // auxiliary state fields first
            dxdt[x_index] = v*std::cos(x[yaw_index]);
            dxdt[y_index] = v*std::sin(x[yaw_index]);
            dxdt[rear_wheel_index] = -v/rr;
            dxdt[pitch_index] = 0; // pitch angle is not integrated and must be obtained using solve_pitch_constraint()

            // state fields
            dxdt[yaw_index] = A(yaw_index_A, steer_index_A)*steer_angle_measurement +
                              A(yaw_index_A, steer_rate_index_A)*steer_rate_measurement;
            dxdt[roll_index] = x[roll_rate_index];
            dxdt[steer_index] = 0; // disable steer angle integration
            dxdt[roll_rate_index] = -(C_01*steer_rate_measurement +
                                      K_01*steer_angle_measurement +
                                      C_00*x[roll_rate_index] +
                                      K_00*x[roll_index])/M_00;
            dxdt[steer_rate_index] = 0; // disable steer rate integration
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place

    // set steer angle and rate to measured values
    xout[index(full_state_index_t::steer_angle)] = steer_angle_measurement;
    xout[index(full_state_index_t::steer_rate)] = steer_rate_measurement;
    return xout;
}

void BicycleArend::set_state_space() {
    Bicycle::set_state_space();
    set_K();
}

void BicycleArend::set_output_element(output_t& x, output_index_t field, real_t value) {
    x[index(field)] = value;
}

real_t BicycleArend::get_output_element(const output_t& x, output_index_t field) {
    return x[index(field)];
}

BicycleArend::output_t BicycleArend::normalize_output(const output_t& y) const {
    static_assert(index(output_index_t::steer_angle) == 0,
        "Invalid underlying value for output index element");
    static_assert(index(output_index_t::steer_rate) == 1,
        "Invalid underlying value for output index element");
    static_assert(index(output_index_t::number_of_types) == 2,
        "Invalid underlying value for output index element");

    output_t normalized_y = y;
    const real_t steer = std::fmod(y[index(output_index_t::steer_angle)], constants::two_pi);
    normalized_y[index(output_index_t::steer_angle)] = steer;
    return normalized_y;
}


void BicycleArend::set_K() {
    m_K = constants::g*m_K0 + m_v*m_v*m_K2;
}


} // namespace model
