#pragma once
#include <thread>
#include <asio.hpp>
#include <asio/serial_port.hpp>

namespace network {
class Serial {
    public:
        Serial(const char* devname, uint32_t baud_rate,
                asio::serial_port_base::parity parity=
                    asio::serial_port_base::parity(asio::serial_port_base::parity::none),
                asio::serial_port_base::character_size character_size=asio::serial_port_base::character_size(8),
                asio::serial_port_base::flow_control flow_control=
                    asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none),
                asio::serial_port_base::stop_bits stop_bits=
                    asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        ~Serial();

        void open(const char* devname, uint32_t baud_rate,
                asio::serial_port_base::parity parity=
                    asio::serial_port_base::parity(asio::serial_port_base::parity::none),
                asio::serial_port_base::character_size character_size=asio::serial_port_base::character_size(8),
                asio::serial_port_base::flow_control flow_control=
                    asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none),
                asio::serial_port_base::stop_bits stop_bits=
                    asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

        void close();
        void async_write(asio::const_buffer buffer);

        bool is_open();
        asio::serial_port_base::baud_rate get_baud_rate();
        asio::serial_port_base::parity get_parity();
        asio::serial_port_base::character_size get_character_size();
        asio::serial_port_base::flow_control get_flow_control();
        asio::serial_port_base::stop_bits get_stop_bits();

        static constexpr size_t buffer_size = 128;

    private:
        asio::io_service m_io_service;
        asio::serial_port m_port;
        std::thread m_service_thread;
        std::array<uint8_t, buffer_size> m_receive_buffer;

        void start_read();
        void handle_read(const asio::error_code& error, size_t bytes_tranferred);
        void handle_write(const asio::error_code& error, size_t bytes_tranferred);

        void run_service();
};

} // namespace network
