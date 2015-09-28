#include <array>
#include <chrono>
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
    const double fs = 200;              // sample rate [Hz]
    const double dt = 1.0/fs;           // sample time [s]
    const double v0 = 5.0;              // forward speed [m/s]
    const size_t N = 1000;                // length of simulation in samples
    //const size_t n = 100;               // length of horizon in samples

    model::Bicycle::state_t x; // roll angle, steer angle, roll rate, steer rate

    /* used for serializing/logging */
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::FlatBufferBuilder log_builder;
    flatbuffers::Offset<fbs::SampleBuffer> sample_locations[N];

    /* used for noise generation */
    std::random_device rd; // used only to seed rng
} // namespace


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0, parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> rn1(0, parameters::defaultvalue::kalman::R(1, 1));

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, v0, dt);
    bicycle.set_C(parameters::defaultvalue::bicycle::C);

    observer::Kalman<model::Bicycle> kalman(bicycle,
            parameters::defaultvalue::kalman::Q,
            parameters::defaultvalue::kalman::R,
            model::Bicycle::state_t::Zero(), // starts at zero state
            parameters::defaultvalue::kalman::P0);

//    controller::Lqr<model::Bicycle> lqr(bicycle,
//            controller::Lqr<model::Bicycle>::state_cost_t::Identity(),
//            0.1 * controller::Lqr<model::Bicycle>::input_cost_t::Identity(),
//            model::Bicycle::state_t::Zero(),
//            n);

    x << 0, 10, 10, 0; // define x0 in degrees
    x *= constants::as_radians; // convert to radians

    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "states are: [roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    std::cout << "simulating discrete time system at constant speed (" <<
        N << " steps at " << fs << " Hz)" << std::endl;
    //std::cout << "with kalman filter and lqr controller..." << std::endl;
    std::cout << "with kalman filter..." << std::endl;

    /* flatbuffer objects must be serialized in depth first pre-order traversal
     * bicycle, kalman, and lqr objects must be serialized first
     */

    size_t current_sample = 0;
    auto bicycle_location = fbs::create_bicycle(builder, bicycle);
    auto kalman_location = fbs::create_kalman(builder, kalman);
    //auto lqr_location = fbs::create_lqr(builder, lqr);
    auto fbs_state = fbs::state(x);
    //auto fbs_input = fbs::input(model::Bicycle::input_t::Zero());
    auto fbs_measurement = fbs::output(model::Bicycle::output_t::Zero());
    builder.Finish(fbs::CreateSample(builder, current_sample,
                bicycle_location, kalman_location, 0,
                &fbs_state, 0, 0, &fbs_measurement));

    auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
    sample_locations[current_sample++] = fbs::CreateSampleBuffer(log_builder, data);

    auto disc_start = std::chrono::system_clock::now();
    for (; current_sample < N; ++current_sample) {
        /* compute control law */
        //auto u = lqr.control_calculate(kalman.x());

        /* system simulate */
        //x = bicycle.x_next(x, u);
        x = bicycle.x_next(x);

        /* measure output with noise */
        auto z = bicycle.y(x);
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        /* observer time/measurement update */
        //kalman.time_update(u);
        kalman.time_update();
        kalman.measurement_update(z);

        builder.Clear();

        kalman_location = fbs::create_kalman(builder, kalman,
                true, true, false, false, true);
        //lqr_location = fbs::create_lqr(builder, lqr,
        //        false, false, true, false, false, true);

        /* skip output as we can get it from state */
        fbs_state = fbs::state(x);
        //fbs_input = fbs::input(u);
        fbs_measurement = fbs::output(z);

        builder.Finish(fbs::CreateSample(builder, current_sample,
                    0, kalman_location, 0, &fbs_state, 0, 0, &fbs_measurement));
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
