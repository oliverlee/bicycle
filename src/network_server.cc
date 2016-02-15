#include <functional>
#include <iostream>
#include "network_server.h"

namespace network {
namespace udp {

Server::Server(uint16_t server_port, uint16_t remote_port) :
    m_remote_endpoint(asio::ip::udp::v4(), remote_port),
    m_server_endpoint(asio::ip::udp::v4(), server_port),
    m_socket(m_io_service, m_server_endpoint),
    m_service_thread(std::bind(&Server::run_service, this)),
    m_pending_receptions(0),
    m_pending_transmissions(0),
    m_receive_count(0),
    m_transmit_count(0) {
        std::cout << "Starting UDP server, receiving on port " << server_port << "\n";
        std::cout << "                  transmitting to port " << remote_port << "\n";
}

Server::~Server() {
    m_io_service.stop();
    m_service_thread.join();
}

asio::ip::udp::endpoint Server::remote_endpoint() const {
    return m_remote_endpoint;

}

asio::ip::udp::endpoint Server::server_endpoint() const {
    return m_server_endpoint;

}

void Server::start_receive() {
    m_socket.async_receive_from(asio::buffer(m_receive_buffer), m_server_endpoint,
            std::bind(&Server::handle_receive,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void Server::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    (void)bytes_transferred;
    {
        std::lock_guard<std::mutex> lock(m_receive_mutex);
        ++m_pending_receptions;
    }

    ++m_receive_count;
    if (!error) {
        std::cout << m_receive_count << ": received ";
        for (size_t i = 0; i < bytes_transferred/sizeof(double); ++i) {
            std::cout << *(reinterpret_cast<double*>(m_receive_buffer.data())+ i) << " ";
        }
        std::cout << "\n";
    } else {
        std::cerr << error.message() << "\n";
    }
    start_receive();

    {
        std::lock_guard<std::mutex> lock(m_receive_mutex);
        --m_pending_receptions;
    }
    m_receive_condition_variable.notify_all();
}

void Server::handle_send(const asio::error_code& error, size_t bytes_transferred) {
    (void)bytes_transferred;
    if (!error) {
        //std::cout << "sent " << bytes_transferred << " bytes\n";
    } else {
        std::cerr << error.message() << "\n";
    }

    {
        std::lock_guard<std::mutex> lock(m_send_mutex);
        --m_pending_transmissions;
    }
    m_send_condition_variable.notify_all();
}

//void Server::async_send(uint8_t* buffer, size_t length) {
//    // TODO: ensure that buffer data is not changed if transmission queued
//    m_socket.async_send_to(asio::buffer(buffer, length), m_remote_endpoint,
//            std::bind(&Server::handle_send,
//                this,
//                std::placeholders::_1,
//                std::placeholders::_2));
//}

void Server::async_send(asio::const_buffer buffer) {
    {
        std::lock_guard<std::mutex> lock(m_send_mutex);
        ++m_pending_transmissions;
    }

    // TODO: ensure that buffer data is not changed if transmission queued
    //size_t bytes_copied = asio::buffer_copy(asio::buffer(m_transmit_buffer), buffer);
    //m_socket.async_send_to(asio::buffer(m_transmit_buffer, bytes_copied),
    m_socket.async_send_to(asio::buffer(buffer),
            m_remote_endpoint,
            std::bind(&Server::handle_send,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void Server::run_service() {
    start_receive();
    try {
        m_io_service.run();
    } catch (std::exception& e) {
        std::cerr << e.what() << "\n";
    }
}

void Server::wait_for_receive_complete() {
    std::unique_lock<std::mutex> lock(m_receive_mutex);
    m_receive_condition_variable.wait(lock, [this]{return m_pending_receptions == 0;});
}

void Server::wait_for_send_complete() {
    std::unique_lock<std::mutex> lock(m_send_mutex);
    m_send_condition_variable.wait(lock, [this]{return m_pending_transmissions == 0;});
}

} // namespace udp
} // namespace network
