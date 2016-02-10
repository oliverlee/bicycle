#pragma once
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>
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
        void wait_for_send_complete();

    private:
        std::array<uint8_t, buffer_size> m_receive_buffer;
        asio::io_service m_io_service;
        asio::ip::udp::endpoint m_remote_endpoint;
        asio::ip::udp::socket m_socket;
        std::thread m_service_thread;

        std::mutex m_send_mutex;
        std::condition_variable m_send_condition_variable;
        uint32_t m_pending_transmissions;

        uint32_t m_receive_count;
        uint32_t m_transmit_count;

        asio::ip::udp::endpoint remote_endpoint() const;

        void start_receive();
        void handle_receive(const asio::error_code& error, size_t bytes_tranferred);
        void handle_send(const asio::error_code& error, size_t bytes_tranferred);

        void run_service();
};

} // namespace udp
} // namespace network
