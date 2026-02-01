/**
 * @file IpcServer.hpp
 * @brief ZeroMQ-based IPC Server for Robot Controller Core
 */

#pragma once

#include <zmq.hpp>
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "Message.hpp"
#include "MessageTypes.hpp"

namespace robot_controller {
namespace ipc {

using json = nlohmann::json;

/**
 * Handler function type for processing messages
 * Takes the request message and returns a response payload
 */
using MessageHandler = std::function<json(const Message&)>;

/**
 * IPC Server using ZeroMQ
 *
 * Provides:
 * - REQ/REP socket for synchronous request-response
 * - PUB socket for broadcasting status updates
 */
class IpcServer {
public:
    /**
     * Constructor
     * @param rep_address Address for REP socket (e.g., "tcp://*:5555")
     * @param pub_address Address for PUB socket (e.g., "tcp://*:5556")
     */
    explicit IpcServer(const std::string& rep_address = "tcp://*:5555",
                       const std::string& pub_address = "tcp://*:5556");

    /**
     * Destructor - stops server if running
     */
    ~IpcServer();

    // Non-copyable
    IpcServer(const IpcServer&) = delete;
    IpcServer& operator=(const IpcServer&) = delete;

    /**
     * Start the server
     * @return true if started successfully
     */
    bool start();

    /**
     * Stop the server
     */
    void stop();

    /**
     * Check if server is running
     */
    bool isRunning() const { return m_running; }

    /**
     * Register a handler for a specific message type
     * @param type Message type to handle
     * @param handler Handler function
     */
    void registerHandler(MessageType type, MessageHandler handler);

    /**
     * Publish a message to all subscribers
     * @param message Message to publish
     */
    void publish(const Message& message);

    /**
     * Publish a status update
     * @param payload Status payload
     */
    void publishStatus(const json& payload);

    /**
     * Get server statistics
     */
    struct Stats {
        uint64_t messages_received = 0;
        uint64_t messages_sent = 0;
        uint64_t errors = 0;
        int64_t start_time = 0;
    };
    Stats getStats() const;

private:
    /**
     * REQ/REP handler thread function
     */
    void repThreadFunc();

    /**
     * Process a received message
     * @param raw_message Raw JSON string
     * @return Response message
     */
    Message processMessage(const std::string& raw_message);

    /**
     * Handle PING message (built-in)
     */
    json handlePing(const Message& request);

    // ZeroMQ context and sockets
    zmq::context_t m_context;
    zmq::socket_t m_rep_socket;
    zmq::socket_t m_pub_socket;

    // Addresses
    std::string m_rep_address;
    std::string m_pub_address;

    // Thread management
    std::thread m_rep_thread;
    std::atomic<bool> m_running{false};

    // Message handlers
    std::unordered_map<MessageType, MessageHandler> m_handlers;
    std::mutex m_handlers_mutex;

    // Statistics
    mutable std::mutex m_stats_mutex;
    Stats m_stats;
};

} // namespace ipc
} // namespace robot_controller
