#include "serial.h"
#include <iomanip>
#include <iostream>
#include <termios.h>

namespace network {
Serial::Serial(const char* devname, uint32_t baud_rate,
        asio::serial_port_base::parity parity,
        asio::serial_port_base::character_size character_size,
        asio::serial_port_base::flow_control flow_control,
        asio::serial_port_base::stop_bits stop_bits) :
    m_port(m_io_service),
    m_service_thread(nullptr) {
    open(devname, baud_rate, parity, character_size, flow_control, stop_bits);
}

Serial::~Serial() {
    close();
}

void Serial::open(const char* devname, uint32_t baud_rate,
        asio::serial_port_base::parity parity,
        asio::serial_port_base::character_size character_size,
        asio::serial_port_base::flow_control flow_control,
        asio::serial_port_base::stop_bits stop_bits) {
    if (m_io_service.stopped()) {
        m_io_service.reset();
    }
    m_port.open(devname);

    asio::error_code error;
    m_port.set_option(asio::serial_port_base::baud_rate(baud_rate), error);
    if (error) {
        asio::serial_port_service::native_handle_type native_handle = m_port.native_handle();
        struct termios options;
        tcgetattr(native_handle, &options);
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);
        tcsetattr(native_handle, TCSANOW, &options);
    }

    m_port.set_option(parity);
    m_port.set_option(character_size);
    m_port.set_option(flow_control);
    m_port.set_option(stop_bits);
    //m_service_thread = std::thread(std::bind(&Serial::run_service, this));
}

void Serial::close() {
    if (m_service_thread.joinable()) {
        m_service_thread.join();
    }
    if (is_open()) {
        m_port.cancel();
        m_port.close();
        m_io_service.stop();
    }
}

void Serial::start_write(asio::const_buffer buffer) {
    asio::async_write(m_port, asio::buffer(buffer),
            std::bind(&Serial::handle_write,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

bool Serial::is_open() {
    return m_port.is_open();
}

asio::serial_port_base::baud_rate Serial::get_baud_rate() {
    asio::serial_port_base::baud_rate option;
    m_port.get_option(option);
    return option;
}

asio::serial_port_base::parity Serial::get_parity() {
    asio::serial_port_base::parity option;
    m_port.get_option(option);
    return option;
}

asio::serial_port_base::character_size Serial::get_character_size() {
    asio::serial_port_base::character_size option;
    m_port.get_option(option);
    return option;
}

asio::serial_port_base::flow_control Serial::get_flow_control() {
    asio::serial_port_base::flow_control option;
    m_port.get_option(option);
    return option;
}

asio::serial_port_base::stop_bits Serial::get_stop_bits() {
    asio::serial_port_base::stop_bits option;
    m_port.get_option(option);
    return option;
}

void Serial::start_read() {
    m_service_thread = std::thread(std::bind(&Serial::run_service, this));
}

void Serial::start_read_imp() {
    m_port.async_read_some(asio::buffer(m_receive_buffer),
            std::bind(&Serial::handle_read,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void Serial::handle_read(const asio::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::cout << std::hex;
        for (size_t i = 0; i < bytes_transferred; ++i) {
            std::cout << std::setfill('0') << std::setw(2) << (int)m_receive_buffer[i] << " ";
        }
        std::cout << "\n";
        std::cout << std::dec;
    } else {
        std::cerr << error.message() << "\n";
    }
    start_read_imp();
}

void Serial::handle_write(const asio::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::cout << "sent " << bytes_transferred << " bytes\n";
    } else {
        std::cerr << error.message() << "\n";
    }
}

void Serial::run_service() {
    start_read_imp();
    try {
        m_io_service.run();
    } catch (std::exception& e) {
        std::cerr << e.what() << "\n";
    }
}

} // namespace
