#include <array>
#include <iostream>
#include <thread>
#include "bicycle.h"
#include "parameters.h"

#include "network_server.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model::Bicycle::state_t, N> discrete_time_system_state_n;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::Bicycle bicycle(v0, dt);
    model::Bicycle::state_t x;
    x << 0, 0, 10, 10, 0; // define in degrees
    x *= constants::as_radians;

    network::udp::Server server;

    for (auto& state: discrete_time_system_state_n) {
        state = bicycle.x_next(x);
        x = state;
        server.async_send(reinterpret_cast<uint8_t*>(x.data()), x.size()*sizeof(double));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
