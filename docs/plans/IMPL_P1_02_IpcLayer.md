# IMPL_P1_02: IPC Layer

| Metadata      | Value                           |
|---------------|---------------------------------|
| Plan ID       | IMPL_P1_02                      |
| Covers Tasks  | P1-04, P1-05, P1-06             |
| Status        | DRAFT                           |
| Version       | 1.0                             |
| Created       | 2026-02-01                      |
| Prerequisites | IMPL_P1_01 completed            |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/PROJECT BLUEPRINT_ BỘ ĐIỀU KHIỂN ROBOT HÀN 6-DOF THƯƠNG MẠI.md` | Hiểu IPC architecture, ZeroMQ patterns |

---

## Prerequisites

Trước khi bắt đầu, đảm bảo:

| Requirement | Check |
|-------------|-------|
| IMPL_P1_01 completed | C++ Core và C# UI build thành công |
| vcpkg packages | cppzmq, zeromq đã installed |
| NuGet packages | NetMQ sẽ được thêm |

### Verification

```powershell
# Verify ZeroMQ installed
C:\vcpkg\vcpkg.exe list | Select-String "zmq"

# Expected:
# cppzmq:x64-windows    4.10.x
# zeromq:x64-windows    4.3.x
```

---

## Overview

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        IPC ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   C# UI (Client)                      C++ Core (Server)         │
│   ┌─────────────────┐                ┌─────────────────┐        │
│   │  IpcClient      │                │  IpcServer      │        │
│   │                 │                │                 │        │
│   │  ┌───────────┐  │   REQ/REP     │  ┌───────────┐  │        │
│   │  │ ReqSocket │◄─┼───(5555)──────┼─►│ RepSocket │  │        │
│   │  └───────────┘  │                │  └───────────┘  │        │
│   │                 │                │                 │        │
│   │  ┌───────────┐  │   PUB/SUB     │  ┌───────────┐  │        │
│   │  │ SubSocket │◄─┼───(5556)──────┼──│ PubSocket │  │        │
│   │  └───────────┘  │                │  └───────────┘  │        │
│   │                 │                │                 │        │
│   └─────────────────┘                └─────────────────┘        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘

REQ/REP: Synchronous request-response (commands, queries)
PUB/SUB: Asynchronous publish-subscribe (status updates)
```

### Message Flow

```
1. UI sends PING         →  Core receives, responds PONG
2. UI sends GET_STATUS   →  Core responds with STATUS
3. Core publishes STATUS →  UI receives via subscription
```

---

## PART A: IPC Protocol Definition (P1-04)

### Step 1: Define Message Types

**File:** `src/core/src/ipc/MessageTypes.hpp`

```powershell
$messageTypesHpp = @"
/**
 * @file MessageTypes.hpp
 * @brief IPC Message type definitions
 */

#pragma once

#include <string>

namespace robot_controller {
namespace ipc {

/**
 * Message types for IPC communication
 */
enum class MessageType {
    // Connection
    PING,
    PONG,

    // Status
    GET_STATUS,
    STATUS,

    // Joint positions
    GET_JOINT_POSITIONS,
    JOINT_POSITIONS,

    // Configuration
    GET_CONFIG,
    CONFIG,
    SET_CONFIG,
    CONFIG_ACK,

    // Commands
    COMMAND,
    COMMAND_ACK,

    // Errors
    ERROR,

    // Unknown
    UNKNOWN
};

/**
 * Convert MessageType to string
 */
inline std::string messageTypeToString(MessageType type) {
    switch (type) {
        case MessageType::PING:                return "PING";
        case MessageType::PONG:                return "PONG";
        case MessageType::GET_STATUS:          return "GET_STATUS";
        case MessageType::STATUS:              return "STATUS";
        case MessageType::GET_JOINT_POSITIONS: return "GET_JOINT_POSITIONS";
        case MessageType::JOINT_POSITIONS:     return "JOINT_POSITIONS";
        case MessageType::GET_CONFIG:          return "GET_CONFIG";
        case MessageType::CONFIG:              return "CONFIG";
        case MessageType::SET_CONFIG:          return "SET_CONFIG";
        case MessageType::CONFIG_ACK:          return "CONFIG_ACK";
        case MessageType::COMMAND:             return "COMMAND";
        case MessageType::COMMAND_ACK:         return "COMMAND_ACK";
        case MessageType::ERROR:               return "ERROR";
        default:                               return "UNKNOWN";
    }
}

/**
 * Convert string to MessageType
 */
inline MessageType stringToMessageType(const std::string& str) {
    if (str == "PING")                return MessageType::PING;
    if (str == "PONG")                return MessageType::PONG;
    if (str == "GET_STATUS")          return MessageType::GET_STATUS;
    if (str == "STATUS")              return MessageType::STATUS;
    if (str == "GET_JOINT_POSITIONS") return MessageType::GET_JOINT_POSITIONS;
    if (str == "JOINT_POSITIONS")     return MessageType::JOINT_POSITIONS;
    if (str == "GET_CONFIG")          return MessageType::GET_CONFIG;
    if (str == "CONFIG")              return MessageType::CONFIG;
    if (str == "SET_CONFIG")          return MessageType::SET_CONFIG;
    if (str == "CONFIG_ACK")          return MessageType::CONFIG_ACK;
    if (str == "COMMAND")             return MessageType::COMMAND;
    if (str == "COMMAND_ACK")         return MessageType::COMMAND_ACK;
    if (str == "ERROR")               return MessageType::ERROR;
    return MessageType::UNKNOWN;
}

} // namespace ipc
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\ipc\MessageTypes.hpp" -Value $messageTypesHpp -Encoding UTF8
Write-Host "[OK] MessageTypes.hpp created" -ForegroundColor Green
```

---

### Step 2: Define Message Structure (C++)

**File:** `src/core/src/ipc/Message.hpp`

```powershell
$messageHpp = @"
/**
 * @file Message.hpp
 * @brief IPC Message structure and serialization
 */

#pragma once

#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include "MessageTypes.hpp"

namespace robot_controller {
namespace ipc {

using json = nlohmann::json;

/**
 * IPC Message structure
 *
 * JSON Format:
 * {
 *   "type": "MESSAGE_TYPE",
 *   "id": "uuid-string",
 *   "timestamp": 1234567890123,
 *   "payload": { ... }
 * }
 */
struct Message {
    MessageType type = MessageType::UNKNOWN;
    std::string id;
    int64_t timestamp = 0;
    json payload;

    /**
     * Create a new message with auto-generated ID and timestamp
     */
    static Message create(MessageType type, const json& payload = {}) {
        Message msg;
        msg.type = type;
        msg.id = generateId();
        msg.timestamp = currentTimestamp();
        msg.payload = payload;
        return msg;
    }

    /**
     * Create a response message (keeps same ID)
     */
    static Message createResponse(const Message& request, MessageType responseType, const json& payload = {}) {
        Message msg;
        msg.type = responseType;
        msg.id = request.id;  // Keep same ID for request-response correlation
        msg.timestamp = currentTimestamp();
        msg.payload = payload;
        return msg;
    }

    /**
     * Serialize to JSON string
     */
    std::string serialize() const {
        json j;
        j["type"] = messageTypeToString(type);
        j["id"] = id;
        j["timestamp"] = timestamp;
        j["payload"] = payload;
        return j.dump();
    }

    /**
     * Deserialize from JSON string
     */
    static Message deserialize(const std::string& jsonStr) {
        Message msg;
        try {
            json j = json::parse(jsonStr);
            msg.type = stringToMessageType(j.value("type", "UNKNOWN"));
            msg.id = j.value("id", "");
            msg.timestamp = j.value("timestamp", 0LL);
            msg.payload = j.value("payload", json::object());
        } catch (const json::exception& e) {
            msg.type = MessageType::ERROR;
            msg.payload = {{"error", "Failed to parse message"}, {"details", e.what()}};
        }
        return msg;
    }

    /**
     * Check if message is valid
     */
    bool isValid() const {
        return type != MessageType::UNKNOWN && !id.empty();
    }

private:
    static std::string generateId() {
        // Simple UUID-like ID generation
        static uint64_t counter = 0;
        auto now = std::chrono::high_resolution_clock::now();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        char buf[64];
        snprintf(buf, sizeof(buf), "%016llx-%08llx",
                 static_cast<unsigned long long>(nanos),
                 static_cast<unsigned long long>(++counter));
        return std::string(buf);
    }

    static int64_t currentTimestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

// JSON serialization support for nlohmann::json
inline void to_json(json& j, const Message& msg) {
    j = json{
        {"type", messageTypeToString(msg.type)},
        {"id", msg.id},
        {"timestamp", msg.timestamp},
        {"payload", msg.payload}
    };
}

inline void from_json(const json& j, Message& msg) {
    msg.type = stringToMessageType(j.value("type", "UNKNOWN"));
    msg.id = j.value("id", "");
    msg.timestamp = j.value("timestamp", 0LL);
    msg.payload = j.value("payload", json::object());
}

} // namespace ipc
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\ipc\Message.hpp" -Value $messageHpp -Encoding UTF8
Write-Host "[OK] Message.hpp created" -ForegroundColor Green
```

---

### Step 3: Define Message Classes (C#)

**File:** `src/ui/RobotController.Common/Messages/IpcMessage.cs`

```powershell
$ipcMessageCs = @"
using System.Text.Json;
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Base class for all IPC messages
/// </summary>
public class IpcMessage
{
    [JsonPropertyName("type")]
    public string Type { get; set; } = string.Empty;

    [JsonPropertyName("id")]
    public string Id { get; set; } = string.Empty;

    [JsonPropertyName("timestamp")]
    public long Timestamp { get; set; }

    [JsonPropertyName("payload")]
    public JsonElement Payload { get; set; }

    /// <summary>
    /// Create a new message with auto-generated ID and timestamp
    /// </summary>
    public static T Create<T>(string type) where T : IpcMessage, new()
    {
        return new T
        {
            Type = type,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };
    }

    /// <summary>
    /// Serialize message to JSON string
    /// </summary>
    public string Serialize()
    {
        return JsonSerializer.Serialize(this, new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            WriteIndented = false
        });
    }

    /// <summary>
    /// Deserialize from JSON string
    /// </summary>
    public static T? Deserialize<T>(string json) where T : IpcMessage
    {
        try
        {
            return JsonSerializer.Deserialize<T>(json, new JsonSerializerOptions
            {
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase
            });
        }
        catch
        {
            return null;
        }
    }

    /// <summary>
    /// Deserialize from JSON string to base IpcMessage
    /// </summary>
    public static IpcMessage? Deserialize(string json)
    {
        return Deserialize<IpcMessage>(json);
    }
}
"@

Set-Content -Path "src\ui\RobotController.Common\Messages\IpcMessage.cs" -Value $ipcMessageCs -Encoding UTF8
Write-Host "[OK] IpcMessage.cs created" -ForegroundColor Green
```

---

### Step 4: Define Specific Message Types (C#)

**File:** `src/ui/RobotController.Common/Messages/MessageTypes.cs`

```powershell
$messageTypesCs = @"
namespace RobotController.Common.Messages;

/// <summary>
/// IPC Message type constants
/// </summary>
public static class MessageTypes
{
    // Connection
    public const string PING = "PING";
    public const string PONG = "PONG";

    // Status
    public const string GET_STATUS = "GET_STATUS";
    public const string STATUS = "STATUS";

    // Joint positions
    public const string GET_JOINT_POSITIONS = "GET_JOINT_POSITIONS";
    public const string JOINT_POSITIONS = "JOINT_POSITIONS";

    // Configuration
    public const string GET_CONFIG = "GET_CONFIG";
    public const string CONFIG = "CONFIG";
    public const string SET_CONFIG = "SET_CONFIG";
    public const string CONFIG_ACK = "CONFIG_ACK";

    // Commands
    public const string COMMAND = "COMMAND";
    public const string COMMAND_ACK = "COMMAND_ACK";

    // Errors
    public const string ERROR = "ERROR";
}
"@

Set-Content -Path "src\ui\RobotController.Common\Messages\MessageTypes.cs" -Value $messageTypesCs -Encoding UTF8
Write-Host "[OK] MessageTypes.cs created" -ForegroundColor Green
```

---

### Step 5: Define Payload Classes (C#)

**File:** `src/ui/RobotController.Common/Messages/Payloads.cs`

```powershell
$payloadsCs = @"
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// PONG response payload
/// </summary>
public class PongPayload
{
    [JsonPropertyName("core_version")]
    public string CoreVersion { get; set; } = string.Empty;

    [JsonPropertyName("uptime_ms")]
    public long UptimeMs { get; set; }
}

/// <summary>
/// STATUS payload (published periodically)
/// </summary>
public class StatusPayload
{
    [JsonPropertyName("state")]
    public string State { get; set; } = "IDLE";

    [JsonPropertyName("mode")]
    public string Mode { get; set; } = "MANUAL";

    [JsonPropertyName("joints")]
    public double[] Joints { get; set; } = new double[6];

    [JsonPropertyName("tcp_position")]
    public double[] TcpPosition { get; set; } = new double[6];

    [JsonPropertyName("errors")]
    public string[] Errors { get; set; } = Array.Empty<string>();

    [JsonPropertyName("homed")]
    public bool Homed { get; set; }

    [JsonPropertyName("enabled")]
    public bool Enabled { get; set; }
}

/// <summary>
/// Joint positions payload
/// </summary>
public class JointPositionsPayload
{
    [JsonPropertyName("joints")]
    public double[] Joints { get; set; } = new double[6];

    [JsonPropertyName("unit")]
    public string Unit { get; set; } = "degrees";
}

/// <summary>
/// Error payload
/// </summary>
public class ErrorPayload
{
    [JsonPropertyName("code")]
    public int Code { get; set; }

    [JsonPropertyName("message")]
    public string Message { get; set; } = string.Empty;

    [JsonPropertyName("details")]
    public string? Details { get; set; }
}

/// <summary>
/// Command payload
/// </summary>
public class CommandPayload
{
    [JsonPropertyName("command")]
    public string Command { get; set; } = string.Empty;

    [JsonPropertyName("parameters")]
    public Dictionary<string, object>? Parameters { get; set; }
}
"@

Set-Content -Path "src\ui\RobotController.Common\Messages\Payloads.cs" -Value $payloadsCs -Encoding UTF8
Write-Host "[OK] Payloads.cs created" -ForegroundColor Green
```

---

## PART B: IPC Server - C++ (P1-05)

### Step 6: Create IPC Server Header

**File:** `src/core/src/ipc/IpcServer.hpp`

```powershell
$ipcServerHpp = @"
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
"@

Set-Content -Path "src\core\src\ipc\IpcServer.hpp" -Value $ipcServerHpp -Encoding UTF8
Write-Host "[OK] IpcServer.hpp created" -ForegroundColor Green
```

---

### Step 7: Create IPC Server Implementation

**File:** `src/core/src/ipc/IpcServer.cpp`

```powershell
$ipcServerCpp = @"
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
"@

Set-Content -Path "src\core\src\ipc\IpcServer.cpp" -Value $ipcServerCpp -Encoding UTF8
Write-Host "[OK] IpcServer.cpp created" -ForegroundColor Green
```

---

### Step 8: Update main.cpp to use IPC Server

**File:** `src/core/src/main.cpp`

```powershell
$mainCpp = @"
/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "logging/Logger.hpp"
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signalHandler(int signal) {
    LOG_INFO("Received signal {}, shutting down...", signal);
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");
    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0 starting...");
    LOG_INFO("========================================");

    // Create IPC server
    robot_controller::ipc::IpcServer server("tcp://*:5555", "tcp://*:5556");

    // Register message handlers
    server.registerHandler(robot_controller::ipc::MessageType::GET_STATUS,
        [](const robot_controller::ipc::Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_STATUS request");
            return {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
        });

    server.registerHandler(robot_controller::ipc::MessageType::GET_JOINT_POSITIONS,
        [](const robot_controller::ipc::Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_JOINT_POSITIONS request");
            return {
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
                {"unit", "degrees"}
            };
        });

    // Start server
    if (!server.start()) {
        LOG_ERROR("Failed to start IPC server");
        return 1;
    }

    LOG_INFO("IPC Server running on ports 5555 (REP) and 5556 (PUB)");
    LOG_INFO("Press Ctrl+C to exit");

    // Main loop - publish status periodically
    int status_interval_ms = 100;  // 10 Hz
    auto last_status = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status);

        if (elapsed.count() >= status_interval_ms) {
            // Publish status
            nlohmann::json status = {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
            server.publishStatus(status);
            last_status = now;
        }

        // Sleep to prevent busy loop
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Shutdown
    LOG_INFO("Stopping server...");
    server.stop();

    LOG_INFO("Robot Controller Core shutdown complete");
    return 0;
}
"@

Set-Content -Path "src\core\src\main.cpp" -Value $mainCpp -Encoding UTF8
Write-Host "[OK] main.cpp updated" -ForegroundColor Green
```

---

### Step 9: Update CMakeLists.txt

**File:** Update `src/core/CMakeLists.txt` to include new files

```powershell
# Read current CMakeLists.txt and update CORE_SOURCES
$cmakeContent = Get-Content -Path "src\core\CMakeLists.txt" -Raw

# Update source files section
$oldSources = @"
set(CORE_SOURCES
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/config/ConfigManager.cpp
    src/robot/RobotModel.cpp
    src/logging/Logger.cpp
)
"@

$newSources = @"
set(CORE_SOURCES
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/config/ConfigManager.cpp
    src/robot/RobotModel.cpp
    src/logging/Logger.cpp
)
"@

# Update headers section
$oldHeaders = @"
set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/robot/RobotModel.hpp
    src/robot/DHParameters.hpp
    src/logging/Logger.hpp
)
"@

$newHeaders = @"
set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/ipc/MessageTypes.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/robot/RobotModel.hpp
    src/robot/DHParameters.hpp
    src/logging/Logger.hpp
)
"@

$cmakeContent = $cmakeContent -replace [regex]::Escape($oldHeaders), $newHeaders
Set-Content -Path "src\core\CMakeLists.txt" -Value $cmakeContent -Encoding UTF8

Write-Host "[OK] CMakeLists.txt updated" -ForegroundColor Green
```

---

### Step 10: Build and Test C++ Core

```powershell
# Navigate to core directory
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"

# Clean and reconfigure
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] CMake configure failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] CMake configured" -ForegroundColor Green

# Build
cmake --build build --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] C++ Core built successfully" -ForegroundColor Green
```

**Validation - Run Core:**
```powershell
# Run the core (in a separate terminal)
.\build\bin\Release\robot_core.exe
```

**Expected Output:**
```
[2026-02-01 ...] [info] ========================================
[2026-02-01 ...] [info] Robot Controller Core v1.0.0 starting...
[2026-02-01 ...] [info] ========================================
[2026-02-01 ...] [info] REP socket bound to tcp://*:5555
[2026-02-01 ...] [info] PUB socket bound to tcp://*:5556
[2026-02-01 ...] [info] IpcServer started successfully
[2026-02-01 ...] [info] IPC Server running on ports 5555 (REP) and 5556 (PUB)
[2026-02-01 ...] [info] Press Ctrl+C to exit
```

---

## PART C: IPC Client - C# (P1-06)

### Step 11: Create IPC Client Interface

**File:** `src/ui/RobotController.UI/Services/IIpcClientService.cs`

```powershell
$ipcClientInterfaceCs = @"
using RobotController.Common.Messages;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for IPC client service
/// </summary>
public interface IIpcClientService : IDisposable
{
    /// <summary>
    /// Connection state
    /// </summary>
    bool IsConnected { get; }

    /// <summary>
    /// Connect to the Core server
    /// </summary>
    Task<bool> ConnectAsync(string reqAddress = "tcp://localhost:5555",
                            string subAddress = "tcp://localhost:5556",
                            CancellationToken cancellationToken = default);

    /// <summary>
    /// Disconnect from the Core server
    /// </summary>
    void Disconnect();

    /// <summary>
    /// Send a PING and wait for PONG
    /// </summary>
    Task<PongPayload?> PingAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Get current status
    /// </summary>
    Task<StatusPayload?> GetStatusAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Get current joint positions
    /// </summary>
    Task<JointPositionsPayload?> GetJointPositionsAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a raw request and get response
    /// </summary>
    Task<IpcMessage?> SendRequestAsync(IpcMessage request, CancellationToken cancellationToken = default);

    /// <summary>
    /// Event raised when status is received via PUB/SUB
    /// </summary>
    event EventHandler<StatusPayload>? StatusReceived;

    /// <summary>
    /// Event raised when connection state changes
    /// </summary>
    event EventHandler<bool>? ConnectionStateChanged;

    /// <summary>
    /// Event raised when an error occurs
    /// </summary>
    event EventHandler<string>? ErrorOccurred;
}
"@

Set-Content -Path "src\ui\RobotController.UI\Services\IIpcClientService.cs" -Value $ipcClientInterfaceCs -Encoding UTF8
Write-Host "[OK] IIpcClientService.cs created" -ForegroundColor Green
```

---

### Step 12: Create IPC Client Implementation

**File:** `src/ui/RobotController.UI/Services/IpcClientService.cs`

```powershell
$ipcClientServiceCs = @"
using System.Text.Json;
using NetMQ;
using NetMQ.Sockets;
using RobotController.Common.Messages;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// IPC Client using NetMQ (ZeroMQ for .NET)
/// </summary>
public class IpcClientService : IIpcClientService
{
    private RequestSocket? _reqSocket;
    private SubscriberSocket? _subSocket;
    private NetMQPoller? _poller;
    private Task? _pollerTask;
    private CancellationTokenSource? _pollerCts;

    private bool _isConnected;
    private bool _disposed;

    private readonly object _sendLock = new();
    private readonly TimeSpan _requestTimeout = TimeSpan.FromSeconds(5);

    public bool IsConnected => _isConnected;

    public event EventHandler<StatusPayload>? StatusReceived;
    public event EventHandler<bool>? ConnectionStateChanged;
    public event EventHandler<string>? ErrorOccurred;

    public async Task<bool> ConnectAsync(string reqAddress = "tcp://localhost:5555",
                                         string subAddress = "tcp://localhost:5556",
                                         CancellationToken cancellationToken = default)
    {
        if (_isConnected)
        {
            Log.Warning("Already connected");
            return true;
        }

        try
        {
            Log.Information("Connecting to Core at REQ={ReqAddr}, SUB={SubAddr}", reqAddress, subAddress);

            // Create and connect REQ socket
            _reqSocket = new RequestSocket();
            _reqSocket.Options.Linger = TimeSpan.Zero;
            _reqSocket.Connect(reqAddress);

            // Create and connect SUB socket
            _subSocket = new SubscriberSocket();
            _subSocket.Options.Linger = TimeSpan.Zero;
            _subSocket.Connect(subAddress);
            _subSocket.SubscribeToAnyTopic();

            // Setup subscriber event handler
            _subSocket.ReceiveReady += OnSubscriberReceiveReady;

            // Start poller for SUB socket
            _pollerCts = new CancellationTokenSource();
            _poller = new NetMQPoller { _subSocket };
            _pollerTask = Task.Run(() =>
            {
                try
                {
                    _poller.Run();
                }
                catch (Exception ex)
                {
                    if (!_pollerCts.IsCancellationRequested)
                    {
                        Log.Error(ex, "Poller error");
                    }
                }
            });

            // Test connection with PING
            var pong = await PingAsync(cancellationToken);
            if (pong == null)
            {
                Log.Error("Failed to receive PONG from Core");
                Disconnect();
                return false;
            }

            _isConnected = true;
            Log.Information("Connected to Core v{Version}, uptime={Uptime}ms",
                pong.CoreVersion, pong.UptimeMs);

            ConnectionStateChanged?.Invoke(this, true);
            return true;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to connect to Core");
            ErrorOccurred?.Invoke(this, $"Connection failed: {ex.Message}");
            Disconnect();
            return false;
        }
    }

    public void Disconnect()
    {
        if (!_isConnected && _reqSocket == null)
        {
            return;
        }

        Log.Information("Disconnecting from Core");

        try
        {
            // Stop poller
            _pollerCts?.Cancel();
            _poller?.Stop();
            _pollerTask?.Wait(TimeSpan.FromSeconds(1));

            // Cleanup SUB socket
            if (_subSocket != null)
            {
                _subSocket.ReceiveReady -= OnSubscriberReceiveReady;
                _subSocket.Dispose();
                _subSocket = null;
            }

            // Cleanup REQ socket
            _reqSocket?.Dispose();
            _reqSocket = null;

            // Cleanup poller
            _poller?.Dispose();
            _poller = null;
            _pollerCts?.Dispose();
            _pollerCts = null;
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "Error during disconnect");
        }

        bool wasConnected = _isConnected;
        _isConnected = false;

        if (wasConnected)
        {
            ConnectionStateChanged?.Invoke(this, false);
        }

        Log.Information("Disconnected from Core");
    }

    public async Task<PongPayload?> PingAsync(CancellationToken cancellationToken = default)
    {
        var request = new IpcMessage
        {
            Type = MessageTypes.PING,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };

        var response = await SendRequestAsync(request, cancellationToken);
        if (response == null || response.Type != MessageTypes.PONG)
        {
            return null;
        }

        try
        {
            return JsonSerializer.Deserialize<PongPayload>(response.Payload.GetRawText());
        }
        catch
        {
            return null;
        }
    }

    public async Task<StatusPayload?> GetStatusAsync(CancellationToken cancellationToken = default)
    {
        var request = new IpcMessage
        {
            Type = MessageTypes.GET_STATUS,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };

        var response = await SendRequestAsync(request, cancellationToken);
        if (response == null || response.Type != MessageTypes.STATUS)
        {
            return null;
        }

        try
        {
            return JsonSerializer.Deserialize<StatusPayload>(response.Payload.GetRawText());
        }
        catch
        {
            return null;
        }
    }

    public async Task<JointPositionsPayload?> GetJointPositionsAsync(CancellationToken cancellationToken = default)
    {
        var request = new IpcMessage
        {
            Type = MessageTypes.GET_JOINT_POSITIONS,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };

        var response = await SendRequestAsync(request, cancellationToken);
        if (response == null || response.Type != MessageTypes.JOINT_POSITIONS)
        {
            return null;
        }

        try
        {
            return JsonSerializer.Deserialize<JointPositionsPayload>(response.Payload.GetRawText());
        }
        catch
        {
            return null;
        }
    }

    public Task<IpcMessage?> SendRequestAsync(IpcMessage request, CancellationToken cancellationToken = default)
    {
        return Task.Run(() =>
        {
            if (_reqSocket == null)
            {
                Log.Warning("Cannot send request: not connected");
                return null;
            }

            lock (_sendLock)
            {
                try
                {
                    // Serialize and send
                    string json = JsonSerializer.Serialize(request);
                    Log.Debug("Sending: {Message}", json);

                    if (!_reqSocket.TrySendFrame(_requestTimeout, json))
                    {
                        Log.Error("Send timeout");
                        return null;
                    }

                    // Receive response
                    if (!_reqSocket.TryReceiveFrameString(_requestTimeout, out string? responseJson))
                    {
                        Log.Error("Receive timeout");
                        return null;
                    }

                    Log.Debug("Received: {Message}", responseJson);

                    // Deserialize
                    return JsonSerializer.Deserialize<IpcMessage>(responseJson);
                }
                catch (Exception ex)
                {
                    Log.Error(ex, "Error sending request");
                    ErrorOccurred?.Invoke(this, $"Request error: {ex.Message}");
                    return null;
                }
            }
        }, cancellationToken);
    }

    private void OnSubscriberReceiveReady(object? sender, NetMQSocketEventArgs e)
    {
        try
        {
            string? message = e.Socket.ReceiveFrameString();
            if (string.IsNullOrEmpty(message))
            {
                return;
            }

            Log.Verbose("SUB received: {Message}", message);

            var ipcMessage = JsonSerializer.Deserialize<IpcMessage>(message);
            if (ipcMessage == null)
            {
                return;
            }

            if (ipcMessage.Type == MessageTypes.STATUS)
            {
                var status = JsonSerializer.Deserialize<StatusPayload>(ipcMessage.Payload.GetRawText());
                if (status != null)
                {
                    StatusReceived?.Invoke(this, status);
                }
            }
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "Error processing SUB message");
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        Disconnect();
        _disposed = true;

        GC.SuppressFinalize(this);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Services\IpcClientService.cs" -Value $ipcClientServiceCs -Encoding UTF8
Write-Host "[OK] IpcClientService.cs created" -ForegroundColor Green
```

---

### Step 13: Update MainViewModel to use IPC

**File:** `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`

```powershell
$mainViewModelCs = @"
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.Windows.Media;
using System.Windows.Threading;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly IIpcClientService _ipcClient;
    private readonly Dispatcher _dispatcher;
    private bool _disposed;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "UNKNOWN";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    [ObservableProperty]
    private double _tcpX;

    [ObservableProperty]
    private double _tcpY;

    [ObservableProperty]
    private double _tcpZ;

    [ObservableProperty]
    private double _tcpRx;

    [ObservableProperty]
    private double _tcpRy;

    [ObservableProperty]
    private double _tcpRz;

    [ObservableProperty]
    private string _coreVersion = "";

    [ObservableProperty]
    private long _coreUptime;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = 0.0 },
        new JointPosition { Name = "J3", Value = 0.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 0.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    public MainViewModel() : this(new IpcClientService())
    {
    }

    public MainViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
        _dispatcher = Dispatcher.CurrentDispatcher;

        // Subscribe to events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
        _ipcClient.ErrorOccurred += OnErrorOccurred;

        // Auto-connect on startup
        _ = ConnectAsync();
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (_ipcClient.IsConnected)
        {
            Log.Information("Already connected");
            return;
        }

        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        bool success = await _ipcClient.ConnectAsync();

        if (success)
        {
            // Get initial status
            var pong = await _ipcClient.PingAsync();
            if (pong != null)
            {
                CoreVersion = pong.CoreVersion;
                CoreUptime = pong.UptimeMs;
            }

            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }
        }
    }

    [RelayCommand]
    private void Disconnect()
    {
        _ipcClient.Disconnect();
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (!_ipcClient.IsConnected)
        {
            return;
        }

        var status = await _ipcClient.GetStatusAsync();
        if (status != null)
        {
            UpdateStatus(status);
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        _dispatcher.InvokeAsync(() => UpdateStatus(status));
    }

    private void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        _dispatcher.InvokeAsync(() =>
        {
            if (isConnected)
            {
                ConnectionStatus = "Connected";
                ConnectionStatusColor = Brushes.LimeGreen;
            }
            else
            {
                ConnectionStatus = "Disconnected";
                ConnectionStatusColor = Brushes.Red;
                RobotState = "UNKNOWN";
            }
        });
    }

    private void OnErrorOccurred(object? sender, string error)
    {
        Log.Error("IPC Error: {Error}", error);
        _dispatcher.InvokeAsync(() =>
        {
            // Could show error in UI
        });
    }

    private void UpdateStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        // Update joint positions
        if (status.Joints.Length >= 6)
        {
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
            }

            // Trigger UI update
            OnPropertyChanged(nameof(JointPositions));
        }

        // Update TCP position
        if (status.TcpPosition.Length >= 6)
        {
            TcpX = status.TcpPosition[0];
            TcpY = status.TcpPosition[1];
            TcpZ = status.TcpPosition[2];
            TcpRx = status.TcpPosition[3];
            TcpRy = status.TcpPosition[4];
            TcpRz = status.TcpPosition[5];
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _ipcClient.StatusReceived -= OnStatusReceived;
        _ipcClient.ConnectionStateChanged -= OnConnectionStateChanged;
        _ipcClient.ErrorOccurred -= OnErrorOccurred;
        _ipcClient.Dispose();

        _disposed = true;
        GC.SuppressFinalize(this);
    }
}

public class JointPosition : ObservableObject
{
    private string _name = string.Empty;
    private double _value;

    public string Name
    {
        get => _name;
        set => SetProperty(ref _name, value);
    }

    public double Value
    {
        get => _value;
        set => SetProperty(ref _value, value);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\ViewModels\MainViewModel.cs" -Value $mainViewModelCs -Encoding UTF8
Write-Host "[OK] MainViewModel.cs updated" -ForegroundColor Green
```

---

### Step 14: Update App.xaml.cs for DI

**File:** `src/ui/RobotController.UI/App.xaml.cs`

```powershell
$appXamlCs = @"
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using RobotController.UI.Views;
using Serilog;
using System.Windows;

namespace RobotController.UI;

public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup Serilog
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("../../logs/ui.log",
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7)
            .CreateLogger();

        Log.Information("========================================");
        Log.Information("Robot Controller UI starting...");
        Log.Information("========================================");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // Services
                services.AddSingleton<IIpcClientService, IpcClientService>();

                // ViewModels
                services.AddSingleton<MainViewModel>();

                // Views
                services.AddSingleton<MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Show main window
        var mainWindow = _host.Services.GetRequiredService<MainWindow>();
        var viewModel = _host.Services.GetRequiredService<MainViewModel>();
        mainWindow.DataContext = viewModel;
        mainWindow.Show();

        Log.Information("UI initialized");
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        if (_host != null)
        {
            // Dispose ViewModel (which disposes IPC client)
            var viewModel = _host.Services.GetService<MainViewModel>();
            viewModel?.Dispose();

            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\App.xaml.cs" -Value $appXamlCs -Encoding UTF8
Write-Host "[OK] App.xaml.cs updated" -ForegroundColor Green
```

---

### Step 15: Update MainWindow.xaml with Connect Button

**File:** Partial update to `src/ui/RobotController.UI/Views/MainWindow.xaml`

```powershell
# Read the existing file
$mainWindowXaml = Get-Content -Path "src\ui\RobotController.UI\Views\MainWindow.xaml" -Raw

# Update the Toolbar section to add Connect/Disconnect buttons
$oldToolbar = @"
        <!-- Toolbar -->
        <ToolBarTray Grid.Row="1" Background="{StaticResource SecondaryBackground}">
            <ToolBar Background="{StaticResource SecondaryBackground}">
                <Button Content="Connect" Margin="2"/>
                <Button Content="Home" Margin="2"/>
                <Separator/>
                <Button Content="Run" Margin="2"/>
                <Button Content="Pause" Margin="2"/>
                <Button Content="Stop" Margin="2"/>
            </ToolBar>
        </ToolBarTray>
"@

$newToolbar = @"
        <!-- Toolbar -->
        <ToolBarTray Grid.Row="1" Background="{StaticResource SecondaryBackground}">
            <ToolBar Background="{StaticResource SecondaryBackground}">
                <Button Content="Connect" Command="{Binding ConnectCommand}" Margin="2"/>
                <Button Content="Disconnect" Command="{Binding DisconnectCommand}" Margin="2"/>
                <Separator/>
                <Button Content="Refresh" Command="{Binding RefreshStatusCommand}" Margin="2"/>
                <Separator/>
                <Button Content="Home" Margin="2" IsEnabled="False"/>
                <Separator/>
                <Button Content="Run" Margin="2" IsEnabled="False"/>
                <Button Content="Pause" Margin="2" IsEnabled="False"/>
                <Button Content="Stop" Margin="2" IsEnabled="False"/>
            </ToolBar>
        </ToolBarTray>
"@

$mainWindowXaml = $mainWindowXaml -replace [regex]::Escape($oldToolbar), $newToolbar
Set-Content -Path "src\ui\RobotController.UI\Views\MainWindow.xaml" -Value $mainWindowXaml -Encoding UTF8

Write-Host "[OK] MainWindow.xaml updated" -ForegroundColor Green
```

---

### Step 16: Build C# Solution

```powershell
# Navigate to UI directory
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"

# Restore and build
dotnet restore
dotnet build --configuration Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] C# build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] C# UI built successfully" -ForegroundColor Green
```

---

## Step 17: Integration Test

### 17.1 Start C++ Core

```powershell
# Terminal 1: Start Core
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"
.\build\bin\Release\robot_core.exe
```

### 17.2 Start C# UI

```powershell
# Terminal 2: Start UI
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"
dotnet run --project RobotController.UI --configuration Release
```

### 17.3 Verification Checklist

| Test | Expected | Pass |
|------|----------|------|
| Core starts | Logs show "IPC Server running" | [ ] |
| UI starts | MainWindow appears | [ ] |
| Auto-connect | Status shows "Connected" (green) | [ ] |
| Status bar | Shows "Connected" with green dot | [ ] |
| Robot State | Shows "IDLE" | [ ] |
| Joint positions | Shows J1-J6 values | [ ] |
| TCP position | Shows X, Y, Z values | [ ] |
| Core logs | Shows "Received: PING" messages | [ ] |
| UI logs | Shows "Connected to Core" | [ ] |
| Disconnect button | Changes status to "Disconnected" | [ ] |
| Reconnect | Status returns to "Connected" | [ ] |
| Ctrl+C Core | Core shuts down gracefully | [ ] |
| UI after Core stop | Status changes to "Disconnected" | [ ] |

---

## Step 18: Update Tests

**File:** `src/core/tests/test_ipc.cpp`

```powershell
$testIpcCpp = @"
/**
 * @file test_ipc.cpp
 * @brief IPC Server tests
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::ipc;

class IpcServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init("test_ipc.log", "debug");
    }

    void TearDown() override {
        // Cleanup
    }
};

TEST_F(IpcServerTest, CreateInstance) {
    IpcServer server("tcp://*:15555", "tcp://*:15556");
    SUCCEED();
}

TEST_F(IpcServerTest, StartStop) {
    IpcServer server("tcp://*:15557", "tcp://*:15558");

    EXPECT_FALSE(server.isRunning());

    EXPECT_TRUE(server.start());
    EXPECT_TRUE(server.isRunning());

    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.stop();
    EXPECT_FALSE(server.isRunning());
}

TEST_F(IpcServerTest, MessageSerialization) {
    Message msg = Message::create(MessageType::PING, {{"test", "value"}});

    EXPECT_EQ(msg.type, MessageType::PING);
    EXPECT_FALSE(msg.id.empty());
    EXPECT_GT(msg.timestamp, 0);

    std::string json = msg.serialize();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("PING"), std::string::npos);
}

TEST_F(IpcServerTest, MessageDeserialization) {
    std::string json = R"({
        "type": "PING",
        "id": "test-123",
        "timestamp": 1234567890,
        "payload": {"key": "value"}
    })";

    Message msg = Message::deserialize(json);

    EXPECT_EQ(msg.type, MessageType::PING);
    EXPECT_EQ(msg.id, "test-123");
    EXPECT_EQ(msg.timestamp, 1234567890);
    EXPECT_TRUE(msg.isValid());
}

TEST_F(IpcServerTest, InvalidMessageDeserialization) {
    std::string invalid_json = "not valid json";

    Message msg = Message::deserialize(invalid_json);

    EXPECT_EQ(msg.type, MessageType::ERROR);
}

TEST_F(IpcServerTest, RegisterHandler) {
    IpcServer server("tcp://*:15559", "tcp://*:15560");

    bool handler_called = false;
    server.registerHandler(MessageType::GET_STATUS, [&handler_called](const Message& req) {
        handler_called = true;
        return nlohmann::json{{"state", "IDLE"}};
    });

    SUCCEED();  // Handler registration doesn't throw
}
"@

Set-Content -Path "src\core\tests\test_ipc.cpp" -Value $testIpcCpp -Encoding UTF8
Write-Host "[OK] test_ipc.cpp updated" -ForegroundColor Green
```

### Build and Run Tests

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"
cmake --build build --config Release --target robot_core_tests
ctest --test-dir build -C Release --output-on-failure

Write-Host "[OK] Tests completed" -ForegroundColor Green
```

---

## Step 19: Git Commit

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

git add .
git commit -m "IMPL_P1_02: IPC Layer complete

- Define IPC protocol (Message format, types)
- Implement C++ IpcServer with ZeroMQ
  - REQ/REP for synchronous requests
  - PUB/SUB for status broadcasting
- Implement C# IpcClientService with NetMQ
  - Auto-connect on startup
  - Status subscription
- Update MainViewModel with IPC integration
- Add Connect/Disconnect/Refresh commands
- Add IPC unit tests

Tasks completed: P1-04, P1-05, P1-06

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Completion Checklist

| Item | Status |
|------|--------|
| MessageTypes.hpp created | [ ] |
| Message.hpp created | [ ] |
| IpcMessage.cs created | [ ] |
| MessageTypes.cs created | [ ] |
| Payloads.cs created | [ ] |
| IpcServer.hpp created | [ ] |
| IpcServer.cpp created | [ ] |
| main.cpp updated | [ ] |
| C++ builds successfully | [ ] |
| C++ tests pass | [ ] |
| IIpcClientService.cs created | [ ] |
| IpcClientService.cs created | [ ] |
| MainViewModel.cs updated | [ ] |
| App.xaml.cs updated | [ ] |
| MainWindow.xaml updated | [ ] |
| C# builds successfully | [ ] |
| Integration test: Core starts | [ ] |
| Integration test: UI connects | [ ] |
| Integration test: Status updates | [ ] |
| Integration test: Disconnect/Reconnect | [ ] |
| Git commit created | [ ] |

---

## Troubleshooting

### Problem: "Address already in use" error

**Cause:** Port 5555 or 5556 already in use.

**Solution:**
```powershell
# Check what's using the port
netstat -ano | findstr ":5555"

# Kill the process (replace PID)
taskkill /PID <PID> /F
```

### Problem: NetMQ not found

**Solution:**
```powershell
cd src\ui
dotnet add RobotController.UI package NetMQ --version 4.0.1.13
dotnet restore
```

### Problem: UI hangs on connect

**Cause:** Core not running or wrong address.

**Solution:**
1. Ensure Core is running first
2. Check firewall settings
3. Verify addresses match (localhost:5555, localhost:5556)

### Problem: Status not updating

**Cause:** SUB socket not receiving.

**Solution:**
1. Check Core is publishing (see logs)
2. Verify SUB socket subscribes to all topics
3. Check NetMQPoller is running

### Problem: "Cannot find type 'JointPosition'" error

**Solution:** Make sure `JointPosition` class is defined in same file or imported.

---

## Next Steps

After completing IMPL_P1_02:
1. Update IMPLEMENTATION_PLAN_TRACKER.md → Mark WRITE-02 as Done
2. Proceed to **IMPL_P1_03: Config & Logging**

---

*Document Version: 1.0 | Created: 2026-02-01*
