#include <cstdlib>
#include <chrono>
#include <iostream>
#include <thread>
#include "serial.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_device> <baud_rate>\n"
            << "\nTest serial communication.\n"
            << "A virtual serial port can be created using socat with:\n"
            << "\t$ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n";
        return EXIT_FAILURE;
    }

    uint32_t baud_rate = std::atoi(argv[2]);
    network::Serial serial(argv[1], baud_rate);
    std::cout << "Connected to serial device: " << argv[1] << " at baud rate: "
        << baud_rate << "\n";

    int i = 0;
    while (true) {
        serial.async_write(asio::buffer(&i, 1));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++i;
    }
}

