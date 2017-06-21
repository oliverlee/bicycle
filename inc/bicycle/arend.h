#pragma once
#include "bicycle.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

namespace model {

class BicycleArend final : public Bicycle {
    public:
        enum class output_index_t: uint8_t {
            steer_angle = 0,
            steer_rate,
            number_of_types
        };

        BicycleArend(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v, real_t dt);
        BicycleArend(const char* param_file, real_t v, real_t dt);
        BicycleArend(real_t v, real_t dt);

        virtual state_t update_state(const state_t& x, const input_t& u, const measurement_t& z) const override;
        virtual full_state_t integrate_full_state(const full_state_t& xf, const input_t& u, real_t t, const measurement_t& z) const override;

        virtual void set_state_space() override;

        // define non-static output element set and get functions due to new output_index_t definition
        void set_output_element(output_t& x, output_index_t field, real_t value);
        real_t get_output_element(const output_t& x, output_index_t field);
        virtual output_t normalize_output(const output_t& y) const override;

    private:
        /*
         * Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we mark them as mutable as we do not perceive the internal
         * state change of the stepper.
         */
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            full_state_t, real_t, full_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper;
        second_order_matrix_t m_K;

        void set_K();
};

} // namespace model
