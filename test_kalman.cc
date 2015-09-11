#include <array>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>
#include "bicycle.h"
#include "kalman.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    model::Bicycle::state_t x;
    const model::Bicycle::output_matrix_t C(
            (model::Bicycle::output_matrix_t() <<
                0, 1, 0, 0,
                0, 0, 1, 0
                ).finished()
            );

    // variance defined in degrees
    const observer::Kalman<model::Bicycle>::error_covariance_t P0(
            observer::Kalman<model::Bicycle>::error_covariance_t::Identity()
            );
    const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q(
            (observer::Kalman<model::Bicycle>::process_noise_covariance_t() <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
                ).finished()
            );
    const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R(
            (observer::Kalman<model::Bicycle>::measurement_noise_covariance_t() <<
                0.001,   0,
                    0, 0.1
                ).finished()
            );

    std::array<model::Bicycle::state_t, N> system_state_true;
    std::array<model::Bicycle::state_t, N> system_state_estimate;
    std::array<model::Bicycle::output_t, N> system_output_estimate;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;

    model::Bicycle bicycle(argv[1], v0, dt);

    x.setZero();
    observer::Kalman<model::Bicycle> kalman(bicycle, Q, R, x, P0);

    x << 0, 5, 5, 0; // define x in degrees
    x = boost::math::constants::degree<double>() * x; // convert to radians
    std::cout << "starting simulation with initial state : [" <<
        x.transpose() << "]'" << std::endl;
    std::cout << "and initial state estimate: [" <<
        kalman.x().transpose() << "]'" << std::endl;

    auto it_x = system_state_true.begin();
    auto it_y = system_output_estimate.begin();
    auto it_xh = system_state_estimate.begin();

    *it_x++ = x;
    *it_y++ = bicycle.y(x);
    *it_xh++ = kalman.x();

    for (; it_x != system_state_true.end(); ++it_x, ++it_y, ++it_xh) {
        // simulate bicycle system
        x = bicycle.x_next(x);
        *it_x = x;
        *it_y = bicycle.y(x);

        // update observer
        kalman.time_update();
        kalman.measurement_update(*it_y);
        *it_xh = kalman.x();
    }

    std::cout << "at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "true state:      [" << x.transpose() << "]'" << std::endl;
    std::cout << "estimated state: [" << kalman.x.transpose() << "]'" << std::endl;

    return EXIT_SUCCESS;
}
