# CORE MODULE: IPC Layer (Inter-Process Communication)

## Document Info
| Item | Value |
|------|-------|
| **Module** | IPC Layer |
| **Layer** | Core Infrastructure |
| **Technology** | ZeroMQ (cppzmq / NetMQ) |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Module IPC Layer cung cấp cơ chế giao tiếp giữa các tầng của hệ thống điều khiển robot:
- **C# UI (WPF)** ↔ **C++ Core Logic**: Command/Response và Status streaming
- **C++ Core** ↔ **Firmware (grblHAL)**: Serial communication (handled by grblHAL module)

Module này đảm bảo độ trễ thấp, throughput cao, và thread-safety cho các ứng dụng real-time.

### 1.2. Key Features
- **ZeroMQ Patterns**: REQ-REP, PUB-SUB, PUSH-PULL
- **Binary Protocol**: Efficient serialization với FlatBuffers/MessagePack
- **Low Latency**: < 1ms round-trip trên localhost
- **Thread-Safe**: Lock-free message passing
- **Fault Tolerant**: Auto-reconnection và heartbeat

### 1.3. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    IPC ARCHITECTURE                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                    C# UI (WPF)                              │ │
│  │                                                             │ │
│  │   ┌──────────────────────────────────────────────────────┐ │ │
│  │   │                    NetMQ Client                       │ │ │
│  │   │                                                       │ │ │
│  │   │  ┌────────────┐  ┌────────────┐  ┌────────────┐     │ │ │
│  │   │  │  Command   │  │   Status   │  │   Event    │     │ │ │
│  │   │  │   Sender   │  │ Subscriber │  │  Handler   │     │ │ │
│  │   │  │  (REQ)     │  │   (SUB)    │  │  (PULL)    │     │ │ │
│  │   │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘     │ │ │
│  │   └────────┼───────────────┼───────────────┼─────────────┘ │ │
│  └────────────┼───────────────┼───────────────┼───────────────┘ │
│               │               │               │                  │
│               │ tcp://        │ tcp://        │ tcp://          │
│               │ :5555         │ :5556         │ :5557           │
│               │               │               │                  │
│  ┌────────────┼───────────────┼───────────────┼───────────────┐ │
│  │            ▼               ▼               ▼                │ │
│  │   ┌────────────┐  ┌────────────┐  ┌────────────┐           │ │
│  │   │  Command   │  │   Status   │  │   Event    │           │ │
│  │   │  Handler   │  │ Publisher  │  │  Sender    │           │ │
│  │   │   (REP)    │  │   (PUB)    │  │  (PUSH)    │           │ │
│  │   └─────┬──────┘  └─────┬──────┘  └─────┬──────┘           │ │
│  │         │               │               │                   │ │
│  │   ┌─────▼───────────────▼───────────────▼─────────────────┐ │ │
│  │   │                 IPC Server (C++)                      │ │ │
│  │   │                                                       │ │ │
│  │   │  ┌─────────────────────────────────────────────────┐ │ │ │
│  │   │  │            Message Router                        │ │ │ │
│  │   │  │                                                  │ │ │ │
│  │   │  │  Commands → StateManager / MotionEngine / etc.  │ │ │ │
│  │   │  │  Status   ← Periodic state broadcasts           │ │ │ │
│  │   │  │  Events   ← Alarms, faults, completions         │ │ │ │
│  │   │  └─────────────────────────────────────────────────┘ │ │ │
│  │   └───────────────────────────────────────────────────────┘ │ │
│  │                                                             │ │
│  │                    C++ Core Logic                           │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. ZeroMQ Patterns

### 2.1. Pattern Selection

```
┌─────────────────────────────────────────────────────────────────┐
│                    ZEROMQ PATTERN USAGE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  PATTERN 1: REQ-REP (Commands)                                  │
│  ════════════════════════════                                   │
│                                                                  │
│     UI (REQ)                           Core (REP)               │
│        │                                  │                      │
│        │──── ServoOn Request ────────────►│                      │
│        │                                  │ Process command      │
│        │◄─── ServoOn Response ───────────│                      │
│        │                                  │                      │
│                                                                  │
│     Use: Synchronous commands requiring confirmation            │
│     Examples: ServoOn, ServoOff, SetMode, StartProgram          │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  PATTERN 2: PUB-SUB (Status)                                    │
│  ════════════════════════════                                   │
│                                                                  │
│     Core (PUB)                         UI (SUB)                 │
│        │                                  │                      │
│        │──── Status @ 100Hz ─────────────►│                      │
│        │──── Status @ 100Hz ─────────────►│                      │
│        │──── Status @ 100Hz ─────────────►│                      │
│        │                                  │                      │
│                                                                  │
│     Use: High-frequency state broadcasting                      │
│     Examples: Joint positions, TCP pose, Servo status           │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  PATTERN 3: PUSH-PULL (Events)                                  │
│  ════════════════════════════                                   │
│                                                                  │
│     Core (PUSH)                        UI (PULL)                │
│        │                                  │                      │
│        │──── Alarm Event ────────────────►│                      │
│        │──── Motion Complete ────────────►│                      │
│        │                                  │                      │
│                                                                  │
│     Use: Guaranteed delivery of important events                │
│     Examples: Alarms, Faults, Program completion                │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Port Allocation

| Port | Pattern | Direction | Purpose |
|------|---------|-----------|---------|
| 5555 | REQ-REP | UI → Core | Command/Response |
| 5556 | PUB-SUB | Core → UI | Status stream |
| 5557 | PUSH-PULL | Core → UI | Events/Alarms |
| 5558 | REQ-REP | Core → UI | UI queries (optional) |

---

## 3. Message Protocol

### 3.1. Message Format

```
┌─────────────────────────────────────────────────────────────────┐
│                    MESSAGE STRUCTURE                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Header (16 bytes)                                         │ │
│  │  ┌──────────┬──────────┬──────────┬──────────────────────┐ │ │
│  │  │ Magic    │ Version  │ Type     │ Payload Length       │ │ │
│  │  │ (4B)     │ (2B)     │ (2B)     │ (4B)                 │ │ │
│  │  │ "ROBO"   │ 0x0001   │ CMD/STS  │ Size in bytes        │ │ │
│  │  └──────────┴──────────┴──────────┴──────────────────────┘ │ │
│  │  ┌──────────────────────────────────────────────────────┐  │ │
│  │  │ Sequence │ Timestamp                                  │  │ │
│  │  │ (4B)     │ (Not in header - in payload if needed)    │  │ │
│  │  └──────────┴────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Payload (Variable length)                                 │ │
│  │                                                            │ │
│  │  Binary serialized data (MessagePack / FlatBuffers)       │ │
│  │                                                            │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2. Message Types

```cpp
// message_types.hpp

#pragma once
#include <cstdint>

namespace robot_controller {
namespace ipc {

/**
 * @brief Message type identifiers
 */
enum class MessageType : uint16_t {
    // Commands (0x01xx)
    CMD_SERVO_ON           = 0x0101,
    CMD_SERVO_OFF          = 0x0102,
    CMD_RESET              = 0x0103,
    CMD_SET_MODE           = 0x0104,
    CMD_JOG_START          = 0x0110,
    CMD_JOG_STOP           = 0x0111,
    CMD_JOG_VELOCITY       = 0x0112,
    CMD_MOVE_LINEAR        = 0x0120,
    CMD_MOVE_JOINT         = 0x0121,
    CMD_MOVE_STOP          = 0x0122,
    CMD_MOVE_PAUSE         = 0x0123,
    CMD_MOVE_RESUME        = 0x0124,
    CMD_PROGRAM_START      = 0x0130,
    CMD_PROGRAM_STOP       = 0x0131,
    CMD_HOME               = 0x0140,
    CMD_SET_TOOL           = 0x0150,
    CMD_SET_WORKFRAME      = 0x0151,

    // Responses (0x02xx)
    RSP_OK                 = 0x0200,
    RSP_ERROR              = 0x0201,
    RSP_BUSY               = 0x0202,
    RSP_REJECTED           = 0x0203,

    // Status (0x03xx)
    STS_SYSTEM             = 0x0300,  // Full system status
    STS_JOINTS             = 0x0301,  // Joint positions only
    STS_TCP                = 0x0302,  // TCP pose only
    STS_SERVO              = 0x0303,  // Servo status
    STS_SAFETY             = 0x0304,  // Safety signals
    STS_WELDING            = 0x0305,  // Welding status
    STS_PROGRAM            = 0x0306,  // Program execution status

    // Events (0x04xx)
    EVT_STATE_CHANGED      = 0x0400,
    EVT_ALARM              = 0x0401,
    EVT_FAULT              = 0x0402,
    EVT_MOTION_COMPLETE    = 0x0410,
    EVT_PROGRAM_COMPLETE   = 0x0411,
    EVT_HOMING_COMPLETE    = 0x0420,
    EVT_LIMIT_TRIGGERED    = 0x0430,
    EVT_ESTOP_ASSERTED     = 0x0431,
    EVT_ESTOP_RELEASED     = 0x0432,

    // Queries (0x05xx)
    QRY_GET_CONFIG         = 0x0500,
    QRY_GET_POSITION       = 0x0501,
    QRY_GET_STATUS         = 0x0502,
};

/**
 * @brief Message header structure
 */
#pragma pack(push, 1)
struct MessageHeader {
    char magic[4];          // "ROBO"
    uint16_t version;       // Protocol version
    uint16_t type;          // MessageType
    uint32_t payload_length;
    uint32_t sequence;      // Message sequence number
};
#pragma pack(pop)

static_assert(sizeof(MessageHeader) == 16, "MessageHeader must be 16 bytes");

/**
 * @brief Response codes
 */
enum class ResponseCode : uint16_t {
    OK              = 0,
    ERROR_UNKNOWN   = 1,
    ERROR_INVALID   = 2,
    ERROR_TIMEOUT   = 3,
    ERROR_BUSY      = 4,
    ERROR_NOT_READY = 5,
    ERROR_SAFETY    = 6,
};

} // namespace ipc
} // namespace robot_controller
```

### 3.3. Status Message Structure

```cpp
// status_messages.hpp

#pragma once
#include <array>
#include <cstdint>

namespace robot_controller {
namespace ipc {

constexpr size_t NUM_JOINTS = 6;

/**
 * @brief System state enumeration (matches StateManager)
 */
enum class SystemStateCode : uint8_t {
    BOOT              = 0,
    ERROR_LOCKOUT     = 1,
    ESTOP_ACTIVE      = 2,
    ESTOP_RESET_NEEDED = 3,
    IDLE              = 4,
    ARMING            = 5,
    OPERATIONAL       = 6,
    STOPPING          = 7,
};

/**
 * @brief Operational mode
 */
enum class OperationalModeCode : uint8_t {
    AUTO = 0,
    T1   = 1,
    T2   = 2,
};

/**
 * @brief Full system status message (sent at ~100Hz)
 */
#pragma pack(push, 1)
struct SystemStatusMessage {
    // Timestamp
    uint64_t timestamp_us;          // Microseconds since epoch

    // System state
    SystemStateCode state;
    OperationalModeCode mode;
    uint8_t servo_enabled;          // bool
    uint8_t program_running;        // bool

    // Joint positions (degrees or radians based on config)
    std::array<double, NUM_JOINTS> joint_positions;
    std::array<double, NUM_JOINTS> joint_velocities;
    std::array<double, NUM_JOINTS> joint_torques;

    // TCP pose
    double tcp_x, tcp_y, tcp_z;     // Position [mm]
    double tcp_rx, tcp_ry, tcp_rz;  // Orientation [deg or rad]

    // TCP velocity
    double tcp_linear_velocity;     // [mm/s]
    double tcp_angular_velocity;    // [deg/s]

    // Safety status (bit flags)
    uint32_t safety_status;
    // Bit 0: E-Stop active
    // Bit 1: Gate open
    // Bit 2: Deadman held
    // Bit 3-8: Limit switches J1-J6
    // Bit 9: Servo fault
    // Bit 10: Following error

    // Motion status
    uint8_t motion_active;
    uint8_t motion_type;            // 0=None, 1=MOVL, 2=MOVJ, 3=Jog
    float motion_progress;          // 0.0 - 1.0

    // Buffer status
    uint16_t planner_buffer_available;
    uint16_t rx_buffer_available;

    // Welding status (if applicable)
    uint8_t arc_on;
    uint8_t gas_on;
    float welding_current;
    float welding_voltage;

    // Program status
    uint32_t program_line;
    uint32_t program_total_lines;
};
#pragma pack(pop)

/**
 * @brief Compact joint-only status (higher frequency)
 */
#pragma pack(push, 1)
struct JointStatusMessage {
    uint64_t timestamp_us;
    std::array<double, NUM_JOINTS> positions;
    std::array<double, NUM_JOINTS> velocities;
    uint8_t state;
};
#pragma pack(pop)

/**
 * @brief Event message
 */
#pragma pack(push, 1)
struct EventMessage {
    uint64_t timestamp_us;
    uint16_t event_type;
    uint32_t event_code;
    uint8_t severity;           // 0=Info, 1=Warning, 2=Error, 3=Critical
    char message[128];          // Human-readable message
};
#pragma pack(pop)

} // namespace ipc
} // namespace robot_controller
```

---

## 4. Task Breakdown

### 4.1. Task List

| ID | Task | Description | Priority | Dependencies |
|----|------|-------------|----------|--------------|
| **T-01** | Message Definitions | Định nghĩa tất cả message types | P0 | None |
| **T-02** | Serialization | MessagePack/Binary serialization | P0 | T-01 |
| **T-03** | IPC Server (C++) | ZeroMQ server với 3 sockets | P0 | T-01, T-02 |
| **T-04** | Command Handler | Xử lý và routing commands | P0 | T-03 |
| **T-05** | Status Publisher | Periodic status broadcasting | P0 | T-03 |
| **T-06** | Event Sender | Push events to UI | P0 | T-03 |
| **T-07** | IPC Client (C#) | NetMQ client wrapper | P0 | T-01 |
| **T-08** | Command Sender | C# command interface | P0 | T-07 |
| **T-09** | Status Subscriber | C# status receiver | P0 | T-07 |
| **T-10** | Event Handler | C# event dispatcher | P0 | T-07 |
| **T-11** | Heartbeat | Connection health monitoring | P1 | T-03, T-07 |
| **T-12** | Reconnection | Auto-reconnect on failure | P1 | T-11 |
| **T-13** | Message Queue | Thread-safe command queue | P0 | T-03 |
| **T-14** | Latency Monitor | Performance monitoring | P1 | T-03, T-07 |
| **T-15** | Unit Tests | IPC communication tests | P0 | T-03, T-07 |

---

## 5. Implementation

### 5.1. IPC Server (C++)

```cpp
// ipc_server.hpp

#pragma once
#include "message_types.hpp"
#include "status_messages.hpp"
#include <zmq.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <queue>
#include <mutex>

namespace robot_controller {
namespace ipc {

/**
 * @brief Command handler callback type
 */
using CommandHandler = std::function<ResponseCode(
    MessageType type,
    const uint8_t* payload,
    size_t payload_size,
    std::vector<uint8_t>& response_payload
)>;

/**
 * @brief IPC Server managing ZeroMQ communication
 *
 * Runs 3 sockets:
 * - REP: Command/Response (synchronous)
 * - PUB: Status broadcasting (periodic)
 * - PUSH: Event delivery (async)
 */
class IPCServer {
public:
    /**
     * @brief Constructor
     * @param cmd_port Command port (REQ-REP)
     * @param status_port Status port (PUB-SUB)
     * @param event_port Event port (PUSH-PULL)
     */
    IPCServer(int cmd_port = 5555,
              int status_port = 5556,
              int event_port = 5557);

    /**
     * @brief Destructor
     */
    ~IPCServer();

    /**
     * @brief Start the IPC server
     */
    void start();

    /**
     * @brief Stop the IPC server
     */
    void stop();

    /**
     * @brief Check if server is running
     */
    bool isRunning() const { return running_; }

    /**
     * @brief Register command handler
     */
    void setCommandHandler(CommandHandler handler) { command_handler_ = handler; }

    /**
     * @brief Publish status message
     * @param status System status to broadcast
     */
    void publishStatus(const SystemStatusMessage& status);

    /**
     * @brief Publish joint-only status (higher frequency)
     */
    void publishJointStatus(const JointStatusMessage& status);

    /**
     * @brief Send event to UI
     */
    void sendEvent(const EventMessage& event);

    /**
     * @brief Send event with parameters
     */
    void sendEvent(MessageType event_type, uint32_t code,
                   uint8_t severity, const std::string& message);

    /**
     * @brief Get statistics
     */
    struct Statistics {
        uint64_t commands_received;
        uint64_t commands_processed;
        uint64_t status_published;
        uint64_t events_sent;
        double avg_command_latency_us;
    };
    Statistics getStatistics() const;

private:
    // ZeroMQ context and sockets
    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> cmd_socket_;    // REP
    std::unique_ptr<zmq::socket_t> status_socket_; // PUB
    std::unique_ptr<zmq::socket_t> event_socket_;  // PUSH

    // Port configuration
    int cmd_port_;
    int status_port_;
    int event_port_;

    // Threading
    std::thread cmd_thread_;
    std::atomic<bool> running_;

    // Command handler
    CommandHandler command_handler_;

    // Statistics
    mutable std::mutex stats_mutex_;
    Statistics stats_;

    // Message sequence
    std::atomic<uint32_t> sequence_counter_;

    // Internal methods
    void cmdThreadFunc();
    void processCommand(const zmq::message_t& request);
    MessageHeader createHeader(MessageType type, size_t payload_size);
    void sendResponse(MessageType type, ResponseCode code,
                      const std::vector<uint8_t>& payload = {});
};

} // namespace ipc
} // namespace robot_controller
```

### 5.2. IPCServer Implementation

```cpp
// ipc_server.cpp

#include "ipc_server.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <cstring>

namespace robot_controller {
namespace ipc {

namespace {
    constexpr char MAGIC[] = "ROBO";
    constexpr uint16_t PROTOCOL_VERSION = 0x0001;

    uint64_t getCurrentTimestamp() {
        auto now = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()
        );
        return static_cast<uint64_t>(us.count());
    }
}

IPCServer::IPCServer(int cmd_port, int status_port, int event_port)
    : cmd_port_(cmd_port)
    , status_port_(status_port)
    , event_port_(event_port)
    , running_(false)
    , sequence_counter_(0)
{
    stats_ = {};
}

IPCServer::~IPCServer() {
    stop();
}

void IPCServer::start() {
    if (running_) return;

    spdlog::info("Starting IPC Server...");

    // Create ZeroMQ context
    context_ = std::make_unique<zmq::context_t>(1);

    // Create REP socket for commands
    cmd_socket_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::rep);
    cmd_socket_->set(zmq::sockopt::rcvtimeo, 100);  // 100ms timeout
    cmd_socket_->bind("tcp://*:" + std::to_string(cmd_port_));
    spdlog::info("Command socket bound to port {}", cmd_port_);

    // Create PUB socket for status
    status_socket_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::pub);
    status_socket_->set(zmq::sockopt::sndhwm, 100);  // High water mark
    status_socket_->bind("tcp://*:" + std::to_string(status_port_));
    spdlog::info("Status socket bound to port {}", status_port_);

    // Create PUSH socket for events
    event_socket_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::push);
    event_socket_->set(zmq::sockopt::sndhwm, 1000);
    event_socket_->bind("tcp://*:" + std::to_string(event_port_));
    spdlog::info("Event socket bound to port {}", event_port_);

    running_ = true;

    // Start command processing thread
    cmd_thread_ = std::thread(&IPCServer::cmdThreadFunc, this);

    spdlog::info("IPC Server started");
}

void IPCServer::stop() {
    if (!running_) return;

    spdlog::info("Stopping IPC Server...");

    running_ = false;

    if (cmd_thread_.joinable()) {
        cmd_thread_.join();
    }

    // Close sockets
    if (cmd_socket_) cmd_socket_->close();
    if (status_socket_) status_socket_->close();
    if (event_socket_) event_socket_->close();

    // Destroy context
    context_.reset();

    spdlog::info("IPC Server stopped");
}

void IPCServer::cmdThreadFunc() {
    while (running_) {
        zmq::message_t request;

        try {
            auto result = cmd_socket_->recv(request, zmq::recv_flags::none);

            if (result) {
                stats_.commands_received++;
                processCommand(request);
            }
        } catch (const zmq::error_t& e) {
            if (e.num() != EAGAIN && running_) {
                spdlog::error("ZMQ error in command thread: {}", e.what());
            }
        }
    }
}

void IPCServer::processCommand(const zmq::message_t& request) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Validate minimum size
    if (request.size() < sizeof(MessageHeader)) {
        spdlog::warn("Invalid message: too small ({})", request.size());
        sendResponse(MessageType::RSP_ERROR, ResponseCode::ERROR_INVALID);
        return;
    }

    // Parse header
    const auto* header = static_cast<const MessageHeader*>(request.data());

    // Validate magic
    if (std::memcmp(header->magic, MAGIC, 4) != 0) {
        spdlog::warn("Invalid message: bad magic");
        sendResponse(MessageType::RSP_ERROR, ResponseCode::ERROR_INVALID);
        return;
    }

    // Validate payload size
    if (request.size() < sizeof(MessageHeader) + header->payload_length) {
        spdlog::warn("Invalid message: payload size mismatch");
        sendResponse(MessageType::RSP_ERROR, ResponseCode::ERROR_INVALID);
        return;
    }

    // Get payload pointer
    const uint8_t* payload = static_cast<const uint8_t*>(request.data()) + sizeof(MessageHeader);
    size_t payload_size = header->payload_length;

    MessageType type = static_cast<MessageType>(header->type);

    spdlog::debug("Received command: type=0x{:04X}, seq={}", header->type, header->sequence);

    // Call handler if registered
    std::vector<uint8_t> response_payload;
    ResponseCode response_code = ResponseCode::ERROR_UNKNOWN;

    if (command_handler_) {
        try {
            response_code = command_handler_(type, payload, payload_size, response_payload);
        } catch (const std::exception& e) {
            spdlog::error("Command handler exception: {}", e.what());
            response_code = ResponseCode::ERROR_UNKNOWN;
        }
    } else {
        spdlog::warn("No command handler registered");
        response_code = ResponseCode::ERROR_UNKNOWN;
    }

    // Send response
    MessageType rsp_type = (response_code == ResponseCode::OK)
                           ? MessageType::RSP_OK
                           : MessageType::RSP_ERROR;
    sendResponse(rsp_type, response_code, response_payload);

    stats_.commands_processed++;

    // Update latency stats
    auto end_time = std::chrono::high_resolution_clock::now();
    auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time
    ).count();

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        // Exponential moving average
        stats_.avg_command_latency_us = stats_.avg_command_latency_us * 0.95 + latency_us * 0.05;
    }
}

void IPCServer::sendResponse(MessageType type, ResponseCode code,
                             const std::vector<uint8_t>& payload) {
    // Build response message
    size_t total_size = sizeof(MessageHeader) + sizeof(uint16_t) + payload.size();
    std::vector<uint8_t> buffer(total_size);

    // Create header
    auto* header = reinterpret_cast<MessageHeader*>(buffer.data());
    std::memcpy(header->magic, MAGIC, 4);
    header->version = PROTOCOL_VERSION;
    header->type = static_cast<uint16_t>(type);
    header->payload_length = sizeof(uint16_t) + payload.size();
    header->sequence = sequence_counter_++;

    // Add response code
    auto* code_ptr = reinterpret_cast<uint16_t*>(buffer.data() + sizeof(MessageHeader));
    *code_ptr = static_cast<uint16_t>(code);

    // Add payload
    if (!payload.empty()) {
        std::memcpy(buffer.data() + sizeof(MessageHeader) + sizeof(uint16_t),
                    payload.data(), payload.size());
    }

    // Send
    zmq::message_t reply(buffer.data(), buffer.size());
    cmd_socket_->send(reply, zmq::send_flags::none);
}

MessageHeader IPCServer::createHeader(MessageType type, size_t payload_size) {
    MessageHeader header;
    std::memcpy(header.magic, MAGIC, 4);
    header.version = PROTOCOL_VERSION;
    header.type = static_cast<uint16_t>(type);
    header.payload_length = static_cast<uint32_t>(payload_size);
    header.sequence = sequence_counter_++;
    return header;
}

void IPCServer::publishStatus(const SystemStatusMessage& status) {
    // Build message
    size_t total_size = sizeof(MessageHeader) + sizeof(SystemStatusMessage);
    std::vector<uint8_t> buffer(total_size);

    auto header = createHeader(MessageType::STS_SYSTEM, sizeof(SystemStatusMessage));
    std::memcpy(buffer.data(), &header, sizeof(header));
    std::memcpy(buffer.data() + sizeof(header), &status, sizeof(status));

    // Publish (non-blocking)
    zmq::message_t msg(buffer.data(), buffer.size());
    status_socket_->send(msg, zmq::send_flags::dontwait);

    stats_.status_published++;
}

void IPCServer::publishJointStatus(const JointStatusMessage& status) {
    size_t total_size = sizeof(MessageHeader) + sizeof(JointStatusMessage);
    std::vector<uint8_t> buffer(total_size);

    auto header = createHeader(MessageType::STS_JOINTS, sizeof(JointStatusMessage));
    std::memcpy(buffer.data(), &header, sizeof(header));
    std::memcpy(buffer.data() + sizeof(header), &status, sizeof(status));

    zmq::message_t msg(buffer.data(), buffer.size());
    status_socket_->send(msg, zmq::send_flags::dontwait);
}

void IPCServer::sendEvent(const EventMessage& event) {
    size_t total_size = sizeof(MessageHeader) + sizeof(EventMessage);
    std::vector<uint8_t> buffer(total_size);

    auto header = createHeader(static_cast<MessageType>(event.event_type),
                               sizeof(EventMessage));
    std::memcpy(buffer.data(), &header, sizeof(header));
    std::memcpy(buffer.data() + sizeof(header), &event, sizeof(event));

    zmq::message_t msg(buffer.data(), buffer.size());
    event_socket_->send(msg, zmq::send_flags::dontwait);

    stats_.events_sent++;
}

void IPCServer::sendEvent(MessageType event_type, uint32_t code,
                          uint8_t severity, const std::string& message) {
    EventMessage event = {};
    event.timestamp_us = getCurrentTimestamp();
    event.event_type = static_cast<uint16_t>(event_type);
    event.event_code = code;
    event.severity = severity;

    // Copy message (truncate if necessary)
    size_t copy_len = std::min(message.size(), sizeof(event.message) - 1);
    std::memcpy(event.message, message.c_str(), copy_len);
    event.message[copy_len] = '\0';

    sendEvent(event);
}

IPCServer::Statistics IPCServer::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

} // namespace ipc
} // namespace robot_controller
```

### 5.3. IPC Client (C#)

```csharp
// IPCClient.cs

using NetMQ;
using NetMQ.Sockets;
using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace RobotController.IPC
{
    /// <summary>
    /// IPC Client for communication with C++ Core
    /// </summary>
    public class IPCClient : IDisposable
    {
        private readonly string _commandEndpoint;
        private readonly string _statusEndpoint;
        private readonly string _eventEndpoint;

        private RequestSocket _commandSocket;
        private SubscriberSocket _statusSocket;
        private PullSocket _eventSocket;

        private Task _statusTask;
        private Task _eventTask;
        private CancellationTokenSource _cts;

        private uint _sequenceCounter = 0;

        public event EventHandler<SystemStatus> StatusReceived;
        public event EventHandler<RobotEvent> EventReceived;
        public event EventHandler<Exception> ErrorOccurred;

        public bool IsConnected { get; private set; }

        public IPCClient(string host = "localhost", int cmdPort = 5555,
                        int statusPort = 5556, int eventPort = 5557)
        {
            _commandEndpoint = $"tcp://{host}:{cmdPort}";
            _statusEndpoint = $"tcp://{host}:{statusPort}";
            _eventEndpoint = $"tcp://{host}:{eventPort}";
        }

        public void Connect()
        {
            try
            {
                // Create and connect sockets
                _commandSocket = new RequestSocket();
                _commandSocket.Connect(_commandEndpoint);
                _commandSocket.Options.ReceiveHighWatermark = 1000;
                _commandSocket.Options.SendTimeout = TimeSpan.FromSeconds(5);
                _commandSocket.Options.ReceiveTimeout = TimeSpan.FromSeconds(5);

                _statusSocket = new SubscriberSocket();
                _statusSocket.Connect(_statusEndpoint);
                _statusSocket.SubscribeToAnyTopic();

                _eventSocket = new PullSocket();
                _eventSocket.Connect(_eventEndpoint);

                _cts = new CancellationTokenSource();

                // Start background receivers
                _statusTask = Task.Run(() => StatusReceiverLoop(_cts.Token));
                _eventTask = Task.Run(() => EventReceiverLoop(_cts.Token));

                IsConnected = true;
            }
            catch (Exception ex)
            {
                ErrorOccurred?.Invoke(this, ex);
                throw;
            }
        }

        public void Disconnect()
        {
            _cts?.Cancel();

            _statusTask?.Wait(TimeSpan.FromSeconds(2));
            _eventTask?.Wait(TimeSpan.FromSeconds(2));

            _commandSocket?.Close();
            _statusSocket?.Close();
            _eventSocket?.Close();

            _commandSocket?.Dispose();
            _statusSocket?.Dispose();
            _eventSocket?.Dispose();

            IsConnected = false;
        }

        #region Command Methods

        public async Task<CommandResponse> SendCommandAsync(MessageType type, byte[] payload = null)
        {
            return await Task.Run(() => SendCommand(type, payload));
        }

        public CommandResponse SendCommand(MessageType type, byte[] payload = null)
        {
            if (!IsConnected)
                throw new InvalidOperationException("Not connected");

            lock (_commandSocket)
            {
                try
                {
                    // Build message
                    var header = CreateHeader(type, payload?.Length ?? 0);
                    var message = new byte[Marshal.SizeOf<MessageHeader>() + (payload?.Length ?? 0)];

                    // Copy header
                    IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf<MessageHeader>());
                    try
                    {
                        Marshal.StructureToPtr(header, ptr, false);
                        Marshal.Copy(ptr, message, 0, Marshal.SizeOf<MessageHeader>());
                    }
                    finally
                    {
                        Marshal.FreeHGlobal(ptr);
                    }

                    // Copy payload
                    if (payload != null)
                    {
                        Array.Copy(payload, 0, message, Marshal.SizeOf<MessageHeader>(), payload.Length);
                    }

                    // Send
                    _commandSocket.SendFrame(message);

                    // Receive response
                    var response = _commandSocket.ReceiveFrameBytes();

                    return ParseResponse(response);
                }
                catch (Exception ex)
                {
                    ErrorOccurred?.Invoke(this, ex);
                    return new CommandResponse { Success = false, ErrorMessage = ex.Message };
                }
            }
        }

        public CommandResponse ServoOn() => SendCommand(MessageType.CMD_SERVO_ON);
        public CommandResponse ServoOff() => SendCommand(MessageType.CMD_SERVO_OFF);
        public CommandResponse Reset() => SendCommand(MessageType.CMD_RESET);

        public CommandResponse SetMode(OperationalMode mode)
        {
            var payload = new byte[] { (byte)mode };
            return SendCommand(MessageType.CMD_SET_MODE, payload);
        }

        public CommandResponse StartJog(int axis, double velocity)
        {
            var payload = new byte[12];
            BitConverter.GetBytes(axis).CopyTo(payload, 0);
            BitConverter.GetBytes(velocity).CopyTo(payload, 4);
            return SendCommand(MessageType.CMD_JOG_START, payload);
        }

        public CommandResponse StopJog() => SendCommand(MessageType.CMD_JOG_STOP);

        public CommandResponse MoveLinear(double[] position, double velocity)
        {
            var payload = new byte[56]; // 6 doubles + 1 double
            for (int i = 0; i < 6; i++)
            {
                BitConverter.GetBytes(position[i]).CopyTo(payload, i * 8);
            }
            BitConverter.GetBytes(velocity).CopyTo(payload, 48);
            return SendCommand(MessageType.CMD_MOVE_LINEAR, payload);
        }

        public CommandResponse MoveJoint(double[] jointAngles, double velocity)
        {
            var payload = new byte[56];
            for (int i = 0; i < 6; i++)
            {
                BitConverter.GetBytes(jointAngles[i]).CopyTo(payload, i * 8);
            }
            BitConverter.GetBytes(velocity).CopyTo(payload, 48);
            return SendCommand(MessageType.CMD_MOVE_JOINT, payload);
        }

        public CommandResponse Stop() => SendCommand(MessageType.CMD_MOVE_STOP);
        public CommandResponse Pause() => SendCommand(MessageType.CMD_MOVE_PAUSE);
        public CommandResponse Resume() => SendCommand(MessageType.CMD_MOVE_RESUME);

        public CommandResponse Home() => SendCommand(MessageType.CMD_HOME);

        #endregion

        #region Background Receivers

        private void StatusReceiverLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    if (_statusSocket.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(100),
                                                           out byte[] frame))
                    {
                        var status = ParseStatusMessage(frame);
                        if (status != null)
                        {
                            StatusReceived?.Invoke(this, status);
                        }
                    }
                }
                catch (Exception ex)
                {
                    if (!token.IsCancellationRequested)
                    {
                        ErrorOccurred?.Invoke(this, ex);
                    }
                }
            }
        }

        private void EventReceiverLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    if (_eventSocket.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(100),
                                                          out byte[] frame))
                    {
                        var evt = ParseEventMessage(frame);
                        if (evt != null)
                        {
                            EventReceived?.Invoke(this, evt);
                        }
                    }
                }
                catch (Exception ex)
                {
                    if (!token.IsCancellationRequested)
                    {
                        ErrorOccurred?.Invoke(this, ex);
                    }
                }
            }
        }

        #endregion

        #region Message Parsing

        private MessageHeader CreateHeader(MessageType type, int payloadLength)
        {
            return new MessageHeader
            {
                Magic = new byte[] { (byte)'R', (byte)'O', (byte)'B', (byte)'O' },
                Version = 0x0001,
                Type = (ushort)type,
                PayloadLength = (uint)payloadLength,
                Sequence = _sequenceCounter++
            };
        }

        private CommandResponse ParseResponse(byte[] data)
        {
            if (data.Length < Marshal.SizeOf<MessageHeader>() + 2)
            {
                return new CommandResponse { Success = false, ErrorMessage = "Invalid response" };
            }

            // Parse response code (after header)
            ushort code = BitConverter.ToUInt16(data, Marshal.SizeOf<MessageHeader>());

            return new CommandResponse
            {
                Success = code == 0,
                ResponseCode = (ResponseCode)code,
                Payload = data.Length > Marshal.SizeOf<MessageHeader>() + 2
                    ? data[(Marshal.SizeOf<MessageHeader>() + 2)..]
                    : null
            };
        }

        private SystemStatus ParseStatusMessage(byte[] data)
        {
            if (data.Length < Marshal.SizeOf<MessageHeader>())
                return null;

            int headerSize = Marshal.SizeOf<MessageHeader>();
            int statusSize = Marshal.SizeOf<SystemStatusNative>();

            if (data.Length < headerSize + statusSize)
                return null;

            // Parse native struct
            IntPtr ptr = Marshal.AllocHGlobal(statusSize);
            try
            {
                Marshal.Copy(data, headerSize, ptr, statusSize);
                var native = Marshal.PtrToStructure<SystemStatusNative>(ptr);
                return SystemStatus.FromNative(native);
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        private RobotEvent ParseEventMessage(byte[] data)
        {
            if (data.Length < Marshal.SizeOf<MessageHeader>())
                return null;

            int headerSize = Marshal.SizeOf<MessageHeader>();

            // Parse event
            ulong timestamp = BitConverter.ToUInt64(data, headerSize);
            ushort eventType = BitConverter.ToUInt16(data, headerSize + 8);
            uint eventCode = BitConverter.ToUInt32(data, headerSize + 10);
            byte severity = data[headerSize + 14];

            // Parse message string
            int messageOffset = headerSize + 15;
            int messageLength = Array.IndexOf(data, (byte)0, messageOffset) - messageOffset;
            if (messageLength < 0) messageLength = data.Length - messageOffset;

            string message = System.Text.Encoding.UTF8.GetString(data, messageOffset, messageLength);

            return new RobotEvent
            {
                Timestamp = DateTimeOffset.FromUnixTimeMilliseconds((long)(timestamp / 1000)).DateTime,
                EventType = (MessageType)eventType,
                EventCode = eventCode,
                Severity = (EventSeverity)severity,
                Message = message
            };
        }

        #endregion

        public void Dispose()
        {
            Disconnect();
        }
    }

    #region Data Types

    public enum MessageType : ushort
    {
        CMD_SERVO_ON = 0x0101,
        CMD_SERVO_OFF = 0x0102,
        CMD_RESET = 0x0103,
        CMD_SET_MODE = 0x0104,
        CMD_JOG_START = 0x0110,
        CMD_JOG_STOP = 0x0111,
        CMD_MOVE_LINEAR = 0x0120,
        CMD_MOVE_JOINT = 0x0121,
        CMD_MOVE_STOP = 0x0122,
        CMD_MOVE_PAUSE = 0x0123,
        CMD_MOVE_RESUME = 0x0124,
        CMD_HOME = 0x0140,

        RSP_OK = 0x0200,
        RSP_ERROR = 0x0201,

        STS_SYSTEM = 0x0300,
        STS_JOINTS = 0x0301,

        EVT_STATE_CHANGED = 0x0400,
        EVT_ALARM = 0x0401,
        EVT_MOTION_COMPLETE = 0x0410,
    }

    public enum ResponseCode : ushort
    {
        OK = 0,
        ErrorUnknown = 1,
        ErrorInvalid = 2,
        ErrorTimeout = 3,
        ErrorBusy = 4,
        ErrorNotReady = 5,
        ErrorSafety = 6,
    }

    public enum OperationalMode : byte
    {
        Auto = 0,
        T1 = 1,
        T2 = 2
    }

    public enum EventSeverity : byte
    {
        Info = 0,
        Warning = 1,
        Error = 2,
        Critical = 3
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct MessageHeader
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] Magic;
        public ushort Version;
        public ushort Type;
        public uint PayloadLength;
        public uint Sequence;
    }

    public class CommandResponse
    {
        public bool Success { get; set; }
        public ResponseCode ResponseCode { get; set; }
        public string ErrorMessage { get; set; }
        public byte[] Payload { get; set; }
    }

    public class SystemStatus
    {
        public DateTime Timestamp { get; set; }
        public SystemState State { get; set; }
        public OperationalMode Mode { get; set; }
        public bool ServoEnabled { get; set; }
        public bool ProgramRunning { get; set; }
        public double[] JointPositions { get; set; }
        public double[] JointVelocities { get; set; }
        public double TcpX { get; set; }
        public double TcpY { get; set; }
        public double TcpZ { get; set; }
        public double TcpRx { get; set; }
        public double TcpRy { get; set; }
        public double TcpRz { get; set; }
        public uint SafetyStatus { get; set; }
        public bool MotionActive { get; set; }
        public float MotionProgress { get; set; }

        public static SystemStatus FromNative(SystemStatusNative native)
        {
            return new SystemStatus
            {
                Timestamp = DateTimeOffset.FromUnixTimeMilliseconds((long)(native.TimestampUs / 1000)).DateTime,
                State = (SystemState)native.State,
                Mode = (OperationalMode)native.Mode,
                ServoEnabled = native.ServoEnabled != 0,
                ProgramRunning = native.ProgramRunning != 0,
                JointPositions = native.JointPositions,
                JointVelocities = native.JointVelocities,
                TcpX = native.TcpX,
                TcpY = native.TcpY,
                TcpZ = native.TcpZ,
                TcpRx = native.TcpRx,
                TcpRy = native.TcpRy,
                TcpRz = native.TcpRz,
                SafetyStatus = native.SafetyStatus,
                MotionActive = native.MotionActive != 0,
                MotionProgress = native.MotionProgress
            };
        }
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct SystemStatusNative
    {
        public ulong TimestampUs;
        public byte State;
        public byte Mode;
        public byte ServoEnabled;
        public byte ProgramRunning;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] JointPositions;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] JointVelocities;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] JointTorques;
        public double TcpX, TcpY, TcpZ;
        public double TcpRx, TcpRy, TcpRz;
        public double TcpLinearVelocity;
        public double TcpAngularVelocity;
        public uint SafetyStatus;
        public byte MotionActive;
        public byte MotionType;
        public float MotionProgress;
        public ushort PlannerBufferAvailable;
        public ushort RxBufferAvailable;
        public byte ArcOn;
        public byte GasOn;
        public float WeldingCurrent;
        public float WeldingVoltage;
        public uint ProgramLine;
        public uint ProgramTotalLines;
    }

    public enum SystemState : byte
    {
        Boot = 0,
        ErrorLockout = 1,
        EstopActive = 2,
        EstopResetNeeded = 3,
        Idle = 4,
        Arming = 5,
        Operational = 6,
        Stopping = 7
    }

    public class RobotEvent
    {
        public DateTime Timestamp { get; set; }
        public MessageType EventType { get; set; }
        public uint EventCode { get; set; }
        public EventSeverity Severity { get; set; }
        public string Message { get; set; }
    }

    #endregion
}
```

---

## 6. Configuration

### 6.1. YAML Configuration

```yaml
# config/ipc_config.yaml

ipc:
  # Server configuration (C++ Core)
  server:
    command_port: 5555
    status_port: 5556
    event_port: 5557
    bind_address: "*"         # Bind to all interfaces

  # Client configuration (C# UI)
  client:
    host: "localhost"         # Or IP address
    command_port: 5555
    status_port: 5556
    event_port: 5557
    connect_timeout_ms: 5000
    request_timeout_ms: 5000

  # Status publishing
  status:
    publish_rate_hz: 100      # 100Hz status updates
    joint_only_rate_hz: 200   # Higher rate for joint-only updates

  # Buffer settings
  buffers:
    send_hwm: 1000            # High water mark for send
    recv_hwm: 1000            # High water mark for receive
    command_queue_size: 100   # Max pending commands

  # Heartbeat
  heartbeat:
    enabled: true
    interval_ms: 1000         # 1 second heartbeat
    timeout_ms: 5000          # 5 second timeout

  # Logging
  logging:
    log_commands: true
    log_status: false         # Too verbose
    log_events: true
    log_latency: true
```

---

## 7. Testing

### 7.1. Unit Tests

```cpp
// test_ipc.cpp

#include <gtest/gtest.h>
#include "ipc/ipc_server.hpp"
#include <zmq.hpp>
#include <thread>
#include <chrono>

using namespace robot_controller::ipc;

class IPCServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        server_ = std::make_unique<IPCServer>(15555, 15556, 15557);

        // Register simple command handler
        server_->setCommandHandler([](MessageType type, const uint8_t* payload,
                                       size_t size, std::vector<uint8_t>& response) {
            return ResponseCode::OK;
        });

        server_->start();

        // Wait for server to be ready
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TearDown() override {
        server_->stop();
    }

    std::unique_ptr<IPCServer> server_;
};

TEST_F(IPCServerTest, ServerStarts) {
    EXPECT_TRUE(server_->isRunning());
}

TEST_F(IPCServerTest, CommandRoundTrip) {
    zmq::context_t ctx(1);
    zmq::socket_t client(ctx, zmq::socket_type::req);
    client.connect("tcp://localhost:15555");

    // Build command
    MessageHeader header;
    std::memcpy(header.magic, "ROBO", 4);
    header.version = 0x0001;
    header.type = static_cast<uint16_t>(MessageType::CMD_SERVO_ON);
    header.payload_length = 0;
    header.sequence = 1;

    zmq::message_t request(&header, sizeof(header));
    client.send(request, zmq::send_flags::none);

    zmq::message_t reply;
    auto result = client.recv(reply);
    EXPECT_TRUE(result.has_value());

    // Verify response
    EXPECT_GE(reply.size(), sizeof(MessageHeader) + sizeof(uint16_t));

    auto* rsp_header = static_cast<const MessageHeader*>(reply.data());
    EXPECT_EQ(rsp_header->type, static_cast<uint16_t>(MessageType::RSP_OK));
}

TEST_F(IPCServerTest, StatusPublishing) {
    zmq::context_t ctx(1);
    zmq::socket_t subscriber(ctx, zmq::socket_type::sub);
    subscriber.connect("tcp://localhost:15556");
    subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::rcvtimeo, 1000);

    // Publish a status
    SystemStatusMessage status = {};
    status.timestamp_us = 12345678;
    status.state = SystemStateCode::IDLE;
    server_->publishStatus(status);

    // Receive it
    zmq::message_t msg;
    auto result = subscriber.recv(msg);
    EXPECT_TRUE(result.has_value());
    EXPECT_GE(msg.size(), sizeof(MessageHeader) + sizeof(SystemStatusMessage));
}

TEST_F(IPCServerTest, EventDelivery) {
    zmq::context_t ctx(1);
    zmq::socket_t puller(ctx, zmq::socket_type::pull);
    puller.connect("tcp://localhost:15557");
    puller.set(zmq::sockopt::rcvtimeo, 1000);

    // Send event
    server_->sendEvent(MessageType::EVT_ALARM, 100, 2, "Test alarm");

    // Receive it
    zmq::message_t msg;
    auto result = puller.recv(msg);
    EXPECT_TRUE(result.has_value());
    EXPECT_GE(msg.size(), sizeof(MessageHeader) + sizeof(EventMessage));
}

TEST_F(IPCServerTest, Latency) {
    zmq::context_t ctx(1);
    zmq::socket_t client(ctx, zmq::socket_type::req);
    client.connect("tcp://localhost:15555");

    MessageHeader header;
    std::memcpy(header.magic, "ROBO", 4);
    header.version = 0x0001;
    header.type = static_cast<uint16_t>(MessageType::CMD_SERVO_ON);
    header.payload_length = 0;
    header.sequence = 0;

    // Measure round-trip time
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100; i++) {
        header.sequence = i;
        zmq::message_t request(&header, sizeof(header));
        client.send(request, zmq::send_flags::none);

        zmq::message_t reply;
        client.recv(reply);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    double avg_latency_us = duration.count() / 100.0;

    // Should be under 1ms average on localhost
    EXPECT_LT(avg_latency_us, 1000.0);
}
```

---

## 8. Performance

### 8.1. Latency Targets

| Metric | Target | Typical |
|--------|--------|---------|
| Command round-trip | < 1 ms | 0.2-0.5 ms |
| Status publish interval | 10 ms (100 Hz) | 10 ms |
| Event delivery | < 5 ms | 1-2 ms |
| Message serialization | < 10 µs | 2-5 µs |

### 8.2. Throughput

| Metric | Target |
|--------|--------|
| Status messages/sec | 100-200 |
| Commands/sec | 1000+ |
| Events/sec | 100+ |

---

## 9. References

### 9.1. External Resources
| Resource | URL |
|----------|-----|
| ZeroMQ Guide | https://zguide.zeromq.org/ |
| cppzmq | https://github.com/zeromq/cppzmq |
| NetMQ | https://github.com/zeromq/netmq |
| MessagePack | https://msgpack.org/ |

---

## APPENDIX

### A. CMake Configuration

```cmake
# CMakeLists.txt addition for IPC

# Find ZeroMQ
find_package(cppzmq REQUIRED)

# IPC module
add_library(ipc
    src/ipc/ipc_server.cpp
    src/ipc/message_types.cpp
)

target_link_libraries(ipc
    PUBLIC
        cppzmq
    PRIVATE
        spdlog::spdlog
)

target_include_directories(ipc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
```

### B. NuGet Packages (C#)

```xml
<!-- packages.config or .csproj -->
<PackageReference Include="NetMQ" Version="4.0.*" />
<PackageReference Include="MessagePack" Version="2.5.*" />
```

### C. Glossary

| Term | Definition |
|------|------------|
| **REQ-REP** | Request-Reply pattern (synchronous) |
| **PUB-SUB** | Publish-Subscribe pattern (broadcast) |
| **PUSH-PULL** | Pipeline pattern (guaranteed delivery) |
| **HWM** | High Water Mark (buffer limit) |
| **Latency** | Time from send to receive |

### D. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
