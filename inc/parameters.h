#pragma once
#include <Eigen/Dense>
#include "bicycle.h"
#include "kalman.h"
#include "constants.h"

namespace parameters {
    namespace benchmark {
        const model::Bicycle::second_order_matrix_t M(
                (model::Bicycle::second_order_matrix_t() <<
                 80.81722, 2.31941332208709, 2.31941332208709, 0.29784188199686).finished());
        const model::Bicycle::second_order_matrix_t C1(
                (model::Bicycle::second_order_matrix_t() <<
                 0.0, 33.86641391492494, -0.85035641456978, 1.6854039739756).finished());
        const model::Bicycle::second_order_matrix_t K0(
                (model::Bicycle::second_order_matrix_t() <<
                 -80.95, -2.59951685249872, -2.59951685249872, -0.80329488458618).finished());
        const model::Bicycle::second_order_matrix_t K2(
                (model::Bicycle::second_order_matrix_t() <<
                 0.0, 76.59734589573222, 0.0, 2.65431523794604).finished());
    } // namespace benchmark

    // default matrices used for examples and tests
    namespace defaultvalue {
        namespace bicycle {
            const model::Bicycle::output_matrix_t C(
                    (model::Bicycle::output_matrix_t() <<
                     0, 1, 0, 0,                // steer angle
                     0, 0, 1, 0).finished());   // roll rate
        } // namespace bicycle

        namespace kalman {
            // numbers given in degrees and then converted to radians
            const observer::Kalman<model::Bicycle>::error_covariance_t P0(
                    10 * observer::Kalman<model::Bicycle>::error_covariance_t::Identity() * constants::as_radians);
            const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q(
                   0.1 * observer::Kalman<model::Bicycle>::process_noise_covariance_t::Identity() * constants::as_radians);
            const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R(
                    (observer::Kalman<model::Bicycle>::measurement_noise_covariance_t() <<
                        0.008,     0,
                           0, 0.01).finished() * constants::as_radians);
        } // namespace kalman
    } // namespace defaultvalue
} // namespace parameters
