#include <cmath>
#include "bicycle/kinematic.h"
#include "constants.h"

namespace {
    template <typename E>
    constexpr uint8_t index(E e) {
        return static_cast<uint8_t>(e);
    }
} // namespace

namespace model {
BicycleKinematic::BicycleKinematic(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt) {
    set_K();
}

BicycleKinematic::BicycleKinematic(const char* param_file, real_t v, real_t dt) :
    Bicycle(param_file, v, dt) {
    set_K();
}

BicycleKinematic::BicycleKinematic(real_t v, real_t dt) :
    Bicycle(v, dt) {
    set_K();
}

BicycleKinematic::state_t BicycleKinematic::update_state(const BicycleKinematic::state_t& x, const BicycleKinematic::input_t& u, const BicycleKinematic::measurement_t& z) const {
    /*
     * Simplified equations of motion are used to simulate the bicycle dynamics.
     * Roll/steer rate and acceleration terms are ignored resulting in:
     *          (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
     *                          [delta] = [T_delta]
     *
     */
    (void)u;
    (void)x;
    const real_t steer_angle_measurement = get_output_element(z, output_index_t::steer_angle);
    const real_t yaw_angle_measurement = get_output_element(z, output_index_t::yaw_angle);

    //static constexpr auto yaw_index_A = index(state_index_t::yaw_angle);
    //static constexpr auto steer_index_A = index(state_index_t::steer_angle);
    const real_t next_roll = -m_K(0, 1)/m_K(0, 0) * steer_angle_measurement;

    state_t next_x = state_t::Zero();
    // FIXME: Don't update yaw because it "looks wrong".
    // steer rate is not used in this simplified update of yaw rate
    //set_state_element(next_x, state_index_t::yaw_angle,
    //        (m_Ad(yaw_index_A, yaw_index_A)*yaw_angle_measurement +
    //         m_Ad(yaw_index_A, steer_index_A)*steer_angle_measurement));
    set_state_element(next_x, state_index_t::steer_angle, steer_angle_measurement);
    set_state_element(next_x, state_index_t::roll_angle, next_roll);
    return next_x;
}

BicycleKinematic::full_state_t BicycleKinematic::integrate_full_state(const BicycleKinematic::full_state_t& xf, const BicycleKinematic::input_t& u, real_t t, const BicycleKinematic::measurement_t& z) const {
    /*
     * As this class is already a simplification, we integrate the auxiliary state part separately, using the state at
     * the previous time. After, integration of the auxiliary state, the dynamic state is updated.
     */
    const real_t steer_angle_measurement = get_output_element(z, output_index_t::steer_angle);
    const real_t yaw_angle_measurement = get_output_element(z, output_index_t::yaw_angle);

    static constexpr auto x_index = index(full_state_index_t::x);
    static constexpr auto y_index = index(full_state_index_t::y);
    static constexpr auto rear_wheel_index = index(full_state_index_t::rear_wheel_angle);
    static constexpr auto pitch_index = index(full_state_index_t::pitch_angle);
    static constexpr auto yaw_index = index(full_state_index_t::yaw_angle);

    const real_t v = m_v;
    const real_t rr = m_rr;
    const state_matrix_t& A = m_A;

    full_state_t xout = xf;
    // this model ignores roll rate and steer rate
    set_full_state_element(xout, full_state_index_t::roll_rate, static_cast<real_t>(0));
    set_full_state_element(xout, full_state_index_t::steer_rate, static_cast<real_t>(0));
    m_stepper.do_step([&A, &u, v, rr](const full_state_t& x, full_state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent;

            dxdt[x_index] = v*std::cos(x[yaw_index]);
            dxdt[y_index] = v*std::sin(x[yaw_index]);
            dxdt[rear_wheel_index] = -v/rr;
            dxdt[pitch_index] = 0; // pitch angle is not integrated and must be obtained using solve_pitch_constraint()
            dxdt.tail<n>() = A*x.tail<n>();
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place

    const real_t next_roll = -m_K(0, 1)/m_K(0, 0) * steer_angle_measurement;
    set_full_state_element(xout, full_state_index_t::roll_angle, next_roll);
    set_full_state_element(xout, full_state_index_t::steer_angle, steer_angle_measurement);
    set_full_state_element(xout, full_state_index_t::roll_rate, static_cast<real_t>(0));
    set_full_state_element(xout, full_state_index_t::steer_rate, static_cast<real_t>(0));
    return xout;
}

void BicycleKinematic::set_state_space() {
    Bicycle::set_state_space();
    set_K();
}

void BicycleKinematic::set_K() {
    m_K = constants::g*m_K0 + m_v*m_v*m_K2;
}


} // namespace model
