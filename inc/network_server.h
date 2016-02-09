#pragma once
#include <array>
#include <iostream>
#include <thread>
#include <asio.hpp>

namespace network {
namespace udp {

constexpr size_t buffer_size = 128;
constexpr uint16_t default_port = 9900;

class Server {
    public:
        Server(uint16_t port=default_port);
        ~Server();
        void async_send(uint8_t* buffer, size_t length);

    private:
        std::array<uint8_t, buffer_size> m_receive_buffer;
        asio::io_service m_io_service;
        asio::ip::udp::endpoint m_remote_endpoint;
        asio::ip::udp::socket m_socket;
        std::thread m_service_thread;

        asio::ip::udp::socket remote_endpoint() const;

        void start_receive();
        void handle_receive(const asio::error_code& error, size_t bytes_tranferred);
        void handle_send() { } // (const asio::error_code& error, size_t bytes_tranferred) { }

        void run_service();
};

} // namespace udp
} // namespace network



