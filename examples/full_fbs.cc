#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>

#include "bicycle.h"
#include "kalman.h"
#include "lqr.h"
#include "parameters.h"

#include "flatbuffers/flatbuffers.h"
#include "sample_log_generated.h"
#include "sample_util.h"


namespace {
    using bicycle_t = model::Bicycle;
    using kalman_t = observer::Kalman<bicycle_t>;
    using lqr_t = controller::Lqr<bicycle_t>;

    constexpr double fs = 200; // sample rate [Hz]
    constexpr double dt = 1.0/fs; // sample time [s]
    constexpr double v0 = 5.0; // forward speed [m/s]
    constexpr size_t N = 1000; // length of simulation in samples
    constexpr size_t n = 100; // length of horizon in samples

    constexpr double q0 = 100000; // roll angle cost weight
    constexpr double q1 = 0; // steer angle cost weight
    constexpr double q2 = q0/10; // roll rate cost weight
    constexpr double q3 = q1/10; // steer rate cost weight

    constexpr double rho = 0.1; // input cost weight scaling constant
    constexpr double r0 = 1; // roll torque cost weight
    constexpr double r1 = 1; // steer torque cost weight

    bicycle_t::state_t x; // roll angle, steer angle, roll rate, steer rate

    /* used for serializing/logging */
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::FlatBufferBuilder log_builder;
    flatbuffers::Offset<fbs::SampleBuffer> sample_locations[N];

    /* reference trajectory (roll angle and roll reference) */
    auto reference = [](double t) {
        const double f = 1; // 1 Hz sine wave
        const bicycle_t::state_t r((bicycle_t::state_t() <<
            3 * std::sin(constants::two_pi*f*t),
            0,
            3*constants::two_pi*f * std::cos(constants::two_pi*f*t),
            0).finished() * constants::as_radians);
        return r;
    };

    /* used for noise generation */
    std::random_device rd; // used only to seed rng
} // namespace


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0, parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> rn1(0, parameters::defaultvalue::kalman::R(1, 1));

    bicycle_t bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v0, dt);
    bicycle.set_C(parameters::defaultvalue::bicycle::C);
    x << 5, 5, 0, 0; // define x0 in degrees
    x *= constants::as_radians; // convert to radians

    kalman_t kalman(bicycle,
            parameters::defaultvalue::kalman::Q(dt),
            parameters::defaultvalue::kalman::R,
            bicycle_t::state_t::Zero(), // starts at zero state
            std::pow(x[0]/2, 2) * bicycle_t::state_matrix_t::Identity());

    lqr_t lqr(bicycle,
            (lqr_t::state_cost_t() <<
             q0,  0,  0,  0,
              0, q1,  0,  0,
              0,  0, q2,  0,
              0,  0,  0, q3).finished() * constants::as_radians,
            (lqr_t::input_cost_t() <<
             r0, 0,
              0, r1).finished() * rho,
            bicycle_t::state_t::Zero(),
            n);

    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "states are: [roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    std::cout << "simulating discrete time system at constant speed (" <<
        N << " steps at " << fs << " Hz)" << std::endl;
    std::cout << "with kalman filter and lqr controller..." << std::endl;

    /* flatbuffer objects must be serialized in depth first pre-order traversal
     * bicycle, kalman, and lqr objects must be serialized first
     */

    size_t current_sample = 0;
    auto bicycle_location = fbs::create_bicycle(builder, bicycle);
    auto kalman_location = fbs::create_kalman(builder, kalman);
    auto lqr_location = fbs::create_lqr(builder, lqr);
    auto fbs_state = fbs::state(x);
    auto fbs_input = fbs::input(bicycle_t::input_t::Zero());
    auto fbs_measurement = fbs::output(bicycle_t::output_t::Zero());
    builder.Finish(fbs::CreateSample(builder, current_sample, 0,
                bicycle_location, kalman_location, lqr_location,
                &fbs_state, 0, 0, &fbs_measurement));

    auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
    sample_locations[current_sample++] = fbs::CreateSampleBuffer(log_builder, data);

    auto disc_start = std::chrono::system_clock::now();
    for (; current_sample < N; ++current_sample) {
        auto comp_start = std::chrono::high_resolution_clock::now();
        /* compute control law */
        auto u = lqr.control_calculate(kalman.x(), reference(dt*current_sample));

        /* system simulate */
        x = bicycle.x_next(x, u);

        /* measure output with noise */
        auto z = bicycle.y(x);
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        /* observer time/measurement update */
        kalman.time_update(u);
        kalman.measurement_update(z);

        builder.Clear();

        kalman_location = fbs::create_kalman(builder, kalman,
                true, true, false, false, true); // x, P, Q, R, K
        lqr_location = fbs::create_lqr(builder, lqr,
                false, true, true, false, false, true); // n, r, P, Q, R, K

        /* skip output as we can get it from state */
        fbs_state = fbs::state(x);
        fbs_input = fbs::input(u);
        fbs_measurement = fbs::output(z);

        auto comp_stop = std::chrono::high_resolution_clock::now();
        auto comp_time = std::chrono::duration<double>(comp_stop - comp_start);
        builder.Finish(fbs::CreateSample(builder, current_sample, comp_time.count(),
                    0, kalman_location, lqr_location,
                    &fbs_state, &fbs_input, 0, &fbs_measurement));
        /* sample is serialized */

        //uint8_t* p = nullptr;
        //auto data = log_builder.CreateUninitializedVector<uint8_t>(builder.GetSize(), &p);
        //std::memcpy(p, builder.GetBufferPointer(), builder.GetSize());
        // Using CreateUninitializedVector occasionally causes the sample to be
        // written incorrectly.
        auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
        sample_locations[current_sample] = fbs::CreateSampleBuffer(log_builder, data);
    }

    auto disc_stop = std::chrono::system_clock::now();
    auto disc_time = disc_stop - disc_start;

    std::cout << "final state: " << x.transpose() << std::endl;
    std::cout << "final state estimate: " << kalman.x().transpose() << std::endl;
    std::cout << "simulation duration: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;

    auto samples_vector = log_builder.CreateVector(sample_locations, N);
    auto log_location = fbs::CreateSampleLog(log_builder, samples_vector);
    log_builder.Finish(log_location, fbs::SampleLogIdentifier());

    {
        disc_start = std::chrono::system_clock::now();
        auto f = std::fopen("samples_full.bin", "wb");
        std::fwrite(log_builder.GetBufferPointer(), sizeof(char), log_builder.GetSize(), f);
        std::fclose(f);
        disc_stop = std::chrono::system_clock::now();
        disc_time = disc_stop - disc_start;
        std::cout << "write to file took: " <<
            std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
            " us" << std::endl;
    }

    return EXIT_SUCCESS;
}
