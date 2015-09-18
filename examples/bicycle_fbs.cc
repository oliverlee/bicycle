#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include "bicycle.h"
#include "parameters.h"

#include "flatbuffers/flatbuffers.h"
#include "sample_log_generated.h"
#include "sample_util.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model::Bicycle::state_t, N> system_state;
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::FlatBufferBuilder log_builder;
    flatbuffers::Offset<fbs::SampleBuffer> sample_locations[N];
} // namespace

flatbuffers::Offset<fbs::Bicycle> create_bicycle(flatbuffers::FlatBufferBuilder& fbb, const model::Bicycle& bicycle) {
    auto M = fbs::second_order_matrix(bicycle.M());
    auto C1 = fbs::second_order_matrix(bicycle.C1());
    auto K0 = fbs::second_order_matrix(bicycle.K0());
    auto K2 = fbs::second_order_matrix(bicycle.K2());
    return fbs::CreateBicycle(fbb, bicycle.v(), bicycle.dt(), &M, &C1, &K0, &K2);
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2, v0, dt);

    model::Bicycle::state_t x; // roll rate, steer rate, roll angle, steer angle
    x << 0, 10, 10, 0; // define x0 in degrees
    x *= boost::math::constants::degree<double>(); // convert to radians
    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout << "states are: [roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    // flatbuffer objects must be serialized in depth first pre-order traversal
    size_t current_sample = 0;

    std::cout << "simulating discrete time system at constant speed (" <<
        N << " steps at " << fs << " Hz) ..." << std::endl;
    auto disc_start = std::chrono::system_clock::now();
    for (auto& state: system_state) {
        x = bicycle.x_next(x);
        state = x;

        builder.Clear();
        fbs::SampleBuilder sample_builder(builder);
        if (current_sample == 0) {
            auto bicycle_location = create_bicycle(builder, bicycle);
            sample_builder.add_bicycle(bicycle_location);
        }
        auto fbs_state = fbs::state(x);
        sample_builder.add_state(&fbs_state);
        sample_builder.add_timestamp(current_sample);
        auto location = sample_builder.Finish();
        builder.Finish(location);
        // sample is serialized

        unsigned char* p = nullptr;
        auto data = log_builder.CreateUninitializedVector<unsigned char>(builder.GetSize(), &p);
        std::memcpy(p, builder.GetBufferPointer(), builder.GetSize());

        fbs::SampleBufferBuilder log_sample_builder(log_builder);
        log_sample_builder.add_data(data);
        sample_locations[current_sample++] = log_sample_builder.Finish();
    }
    auto disc_stop = std::chrono::system_clock::now();
    auto disc_time = disc_stop - disc_start;

    std::cout << "final state: " << system_state.back().transpose() << std::endl;
    std::cout << "simulation duration: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;

    auto samples_vector = log_builder.CreateVector(sample_locations, N);
    auto log_location = fbs::CreateSampleLog(log_builder, samples_vector);
    log_builder.Finish(log_location, fbs::SampleLogIdentifier());

    {
        disc_start = std::chrono::system_clock::now();
        auto f = std::fopen("samples.bin", "wb");
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
