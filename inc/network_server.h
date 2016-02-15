#pragma once
#include <array>
#include <condition_variable>
#include <chrono>
#include <mutex>
#include <thread>
#include <type_traits>
#include <asio.hpp>
#include <asio/high_resolution_timer.hpp>

namespace network {
namespace udp {

constexpr size_t buffer_size = 128;
constexpr uint16_t default_remote_port = 9901;
constexpr uint16_t default_server_port = 9900;

class Server {
    public:
        Server(uint16_t server_port=default_server_port, uint16_t remote_port=default_remote_port);
        ~Server();
        //void async_send(uint8_t* buffer, size_t length);
        void async_send(asio::const_buffer buffer);
        void wait_for_receive_complete();
        void wait_for_send_complete();

    protected:
        asio::io_service m_io_service;

    private:
        std::array<uint8_t, buffer_size> m_receive_buffer;
        //std::array<uint8_t, buffer_size> m_transmit_buffer;
        asio::ip::udp::endpoint m_remote_endpoint;
        asio::ip::udp::endpoint m_server_endpoint;
        asio::ip::udp::socket m_socket;
        std::thread m_service_thread;

        std::mutex m_receive_mutex;
        std::condition_variable m_receive_condition_variable;
        uint32_t m_pending_receptions;

        std::mutex m_send_mutex;
        std::condition_variable m_send_condition_variable;
        uint32_t m_pending_transmissions;

        uint32_t m_receive_count;
        uint32_t m_transmit_count;

        asio::ip::udp::endpoint remote_endpoint() const;
        asio::ip::udp::endpoint server_endpoint() const;

        void start_receive();
        void handle_receive(const asio::error_code& error, size_t bytes_tranferred);
        void handle_send(const asio::error_code& error, size_t bytes_tranferred);

        void run_service();
};

template <typename F>
class PeriodicTransmitServer : public Server {
    static_assert(std::chrono::steady_clock::is_steady, "std::chrono::steady_clock is not steady");
    public:
        using deadline_timer = std::conditional<std::chrono::high_resolution_clock::is_steady,
              asio::high_resolution_timer, asio::steady_timer>::type;

        // period must convert to an integer number of nanoseconds
        PeriodicTransmitServer(uint16_t server_port, uint16_t remote_port,
                std::chrono::nanoseconds deadline_period, F buffer_function) :
            Server(server_port, remote_port),
            m_deadline(deadline_period),
            m_transmit_time(deadline_timer::clock_type::now()),
            m_timer(m_io_service, m_deadline),
            m_buffer_function(buffer_function) {
                m_timer.async_wait(std::bind(&PeriodicTransmitServer::periodic_function, this));
        }

        deadline_timer::clock_type::time_point last_transmit_time() const {
            return m_transmit_time;
        }

    private:
        std::chrono::nanoseconds m_deadline;
        deadline_timer::clock_type::time_point m_transmit_time;
        deadline_timer m_timer;
        F m_buffer_function;

        void periodic_function() {
            async_send(m_buffer_function());
            m_transmit_time = deadline_timer::clock_type::now();
            m_timer.expires_at(m_timer.expires_at() + m_deadline);
            m_timer.async_wait(std::bind(&PeriodicTransmitServer::periodic_function, this));
        }
};

} // namespace udp
} // namespace network
