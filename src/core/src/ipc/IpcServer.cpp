/**
 * @file IpcServer.cpp
 * @brief IPC Server implementation
 */

#include "IpcServer.hpp"
#include "../logging/Logger.hpp"
#include <chrono>

namespace robot_controller {
namespace ipc {

IpcServer::IpcServer(const std::string& rep_address, const std::string& pub_address)
    : m_context(1)
    , m_rep_socket(m_context, zmq::socket_type::rep)
    , m_pub_socket(m_context, zmq::socket_type::pub)
    , m_rep_address(rep_address)
    , m_pub_address(pub_address)
{
    LOG_DEBUG("IpcServer created with REP={}, PUB={}", rep_address, pub_address);

    // Register built-in PING handler
    registerHandler(MessageType::PING, [this](const Message& req) {
        return handlePing(req);
    });
}

IpcServer::~IpcServer() {
    stop();
}

bool IpcServer::start() {
    if (m_running) {
        LOG_WARN("IpcServer already running");
        return true;
    }

    try {
        // Configure sockets
        int linger = 0;
        m_rep_socket.set(zmq::sockopt::linger, linger);
        m_pub_socket.set(zmq::sockopt::linger, linger);

        // Set receive timeout for graceful shutdown
        int timeout = 100;  // 100ms
        m_rep_socket.set(zmq::sockopt::rcvtimeo, timeout);

        // Bind sockets
        m_rep_socket.bind(m_rep_address);
        LOG_INFO("REP socket bound to {}", m_rep_address);

        m_pub_socket.bind(m_pub_address);
        LOG_INFO("PUB socket bound to {}", m_pub_address);

        // Record start time
        {
            std::lock_guard<std::mutex> lock(m_stats_mutex);
            m_stats.start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }

        // Start REP handler thread
        m_running = true;
        m_rep_thread = std::thread(&IpcServer::repThreadFunc, this);

        LOG_INFO("IpcServer started successfully");
        return true;

    } catch (const zmq::error_t& e) {
        LOG_ERROR("Failed to start IpcServer: {}", e.what());
        return false;
    }
}

void IpcServer::stop() {
    if (!m_running) {
        return;
    }

    LOG_INFO("Stopping IpcServer...");
    m_running = false;

    // Wait for thread to finish
    if (m_rep_thread.joinable()) {
        m_rep_thread.join();
    }

    // Close sockets
    try {
        m_rep_socket.close();
        m_pub_socket.close();
    } catch (const zmq::error_t& e) {
        LOG_WARN("Error closing sockets: {}", e.what());
    }

    LOG_INFO("IpcServer stopped");
}

void IpcServer::registerHandler(MessageType type, MessageHandler handler) {
    std::lock_guard<std::mutex> lock(m_handlers_mutex);
    m_handlers[type] = std::move(handler);
    LOG_DEBUG("Registered handler for message type: {}", messageTypeToString(type));
}

void IpcServer::publish(const Message& message) {
    if (!m_running) {
        return;
    }

    try {
        std::string data = message.serialize();
        zmq::message_t zmq_msg(data.data(), data.size());
        m_pub_socket.send(zmq_msg, zmq::send_flags::dontwait);

        {
            std::lock_guard<std::mutex> lock(m_stats_mutex);
            m_stats.messages_sent++;
        }

        LOG_TRACE("Published message type: {}", messageTypeToString(message.type));

    } catch (const zmq::error_t& e) {
        LOG_ERROR("Failed to publish message: {}", e.what());
        std::lock_guard<std::mutex> lock(m_stats_mutex);
        m_stats.errors++;
    }
}

void IpcServer::publishStatus(const json& payload) {
    Message msg = Message::create(MessageType::STATUS, payload);
    publish(msg);
}

IpcServer::Stats IpcServer::getStats() const {
    std::lock_guard<std::mutex> lock(m_stats_mutex);
    return m_stats;
}

void IpcServer::repThreadFunc() {
    LOG_DEBUG("REP handler thread started");

    while (m_running) {
        try {
            zmq::message_t request;

            // Receive with timeout (allows checking m_running flag)
            auto result = m_rep_socket.recv(request, zmq::recv_flags::none);

            if (!result) {
                // Timeout - no message received
                continue;
            }

            // Process message
            std::string raw_message(static_cast<char*>(request.data()), request.size());
            LOG_TRACE("Received: {}", raw_message);

            {
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.messages_received++;
            }

            // Process and get response
            Message response = processMessage(raw_message);

            // Send response
            std::string response_data = response.serialize();
            zmq::message_t reply(response_data.data(), response_data.size());
            m_rep_socket.send(reply, zmq::send_flags::none);

            {
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.messages_sent++;
            }

            LOG_TRACE("Sent response: {}", messageTypeToString(response.type));

        } catch (const zmq::error_t& e) {
            if (m_running) {
                LOG_ERROR("ZMQ error in REP thread: {}", e.what());
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.errors++;
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in REP thread: {}", e.what());
            std::lock_guard<std::mutex> lock(m_stats_mutex);
            m_stats.errors++;
        }
    }

    LOG_DEBUG("REP handler thread stopped");
}

Message IpcServer::processMessage(const std::string& raw_message) {
    // Deserialize
    Message request = Message::deserialize(raw_message);

    if (!request.isValid()) {
        LOG_WARN("Received invalid message");
        return Message::create(MessageType::ERROR, {
            {"code", 400},
            {"message", "Invalid message format"}
        });
    }

    LOG_DEBUG("Processing message type: {}", messageTypeToString(request.type));

    // Find handler
    MessageHandler handler;
    {
        std::lock_guard<std::mutex> lock(m_handlers_mutex);
        auto it = m_handlers.find(request.type);
        if (it != m_handlers.end()) {
            handler = it->second;
        }
    }

    if (handler) {
        try {
            json response_payload = handler(request);

            // Determine response type based on request type
            MessageType response_type;
            switch (request.type) {
                case MessageType::PING:
                    response_type = MessageType::PONG;
                    break;
                case MessageType::GET_STATUS:
                    response_type = MessageType::STATUS;
                    break;
                case MessageType::GET_JOINT_POSITIONS:
                    response_type = MessageType::JOINT_POSITIONS;
                    break;
                case MessageType::GET_CONFIG:
                    response_type = MessageType::CONFIG;
                    break;
                case MessageType::SET_CONFIG:
                    response_type = MessageType::CONFIG_ACK;
                    break;
                case MessageType::COMMAND:
                    response_type = MessageType::COMMAND_ACK;
                    break;
                default:
                    response_type = request.type;
            }

            return Message::createResponse(request, response_type, response_payload);

        } catch (const std::exception& e) {
            LOG_ERROR("Handler error for {}: {}", messageTypeToString(request.type), e.what());
            return Message::createResponse(request, MessageType::ERROR, {
                {"code", 500},
                {"message", "Handler error"},
                {"details", e.what()}
            });
        }
    } else {
        LOG_WARN("No handler for message type: {}", messageTypeToString(request.type));
        return Message::createResponse(request, MessageType::ERROR, {
            {"code", 404},
            {"message", "No handler for message type"},
            {"type", messageTypeToString(request.type)}
        });
    }
}

json IpcServer::handlePing(const Message& request) {
    auto stats = getStats();
    int64_t uptime = 0;
    if (stats.start_time > 0) {
        int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        uptime = now - stats.start_time;
    }

    return {
        {"core_version", "1.0.0"},
        {"uptime_ms", uptime},
        {"stats", {
            {"messages_received", stats.messages_received},
            {"messages_sent", stats.messages_sent},
            {"errors", stats.errors}
        }}
    };
}

} // namespace ipc
} // namespace robot_controller
