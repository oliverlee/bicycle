#pragma once
#include <Eigen/Dense>
#include "bicycle.h"
#include "kalman.h"
#include "constants.h"

namespace parameters {
    namespace benchmark {
        extern const model::Bicycle::second_order_matrix_t M;
        extern const model::Bicycle::second_order_matrix_t C1;
        extern const model::Bicycle::second_order_matrix_t K0;
        extern const model::Bicycle::second_order_matrix_t K2;
        extern const double wheelbase;
        extern const double trail;
        extern const double steer_axis_tilt;
        extern const double rear_wheel_radius;
        extern const double front_wheel_radius;
    } // namespace benchmark

    // default matrices used for examples and tests
    namespace defaultvalue {
        namespace bicycle {
            extern const model::Bicycle::output_matrix_t C;
        } // namespace bicycle

        namespace kalman {
            // numbers given in degrees and then converted to radians
            extern const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q(double dt);
            extern const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R;
        } // namespace kalman
    } // namespace defaultvalue
} // namespace parameters
