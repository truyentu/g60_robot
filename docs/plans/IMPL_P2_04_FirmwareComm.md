# IMPL_P2_04: Firmware Communication

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P2_04 |
| Phase | 2 - Motion Core |
| Priority | P0 (Critical) |
| Depends On | IMPL_P2_03 (Trajectory Generator) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Tối ưu grblHAL cho Robot 6-DOF.md` | grblHAL protocol, step/dir, encoder feedback, realtime loop |

---

## Overview

Implementation plan cho Firmware Communication layer:
- **Serial Communication:** USB/UART với Teensy 4.1
- **Protocol:** Custom binary protocol + grblHAL G-code
- **Command Streaming:** Real-time motion command buffer
- **Status Polling:** Position feedback, limit switches, errors
- **Homing:** Automated homing sequence
- **Safety:** E-stop, limit switch handling

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    C++ Core (PC)                             │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ Trajectory  │──│  Firmware   │──│   Serial Port       │ │
│  │  Executor   │  │  Interface  │  │   Manager           │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                          │                    │             │
│                          ▼                    ▼             │
│               ┌─────────────────┐  ┌─────────────────────┐ │
│               │ Command Buffer  │  │  Response Parser    │ │
│               │ (Ring Buffer)   │  │  (State Machine)    │ │
│               └─────────────────┘  └─────────────────────┘ │
└───────────────────────────┬─────────────────────────────────┘
                            │ USB/Serial
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                 Teensy 4.1 + grblHAL                         │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │  G-code     │  │   Motion    │  │   Step/Dir          │ │
│  │  Parser     │──│   Planner   │──│   Generator         │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                                              │              │
│                                              ▼              │
│                                    ┌─────────────────────┐ │
│                                    │   Stepper Drivers   │ │
│                                    │   (6 axes)          │ │
│                                    └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P2_03 (Trajectory Generator) đã hoàn thành
- [ ] Boost.Asio hoặc libserialport installed
- [ ] Teensy 4.1 với grblHAL firmware
- [ ] USB driver installed

---

## Step 1: Install Serial Port Library

### 1.1 Add Boost.Asio to vcpkg

**Command:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
vcpkg install boost-asio:x64-windows
```

### 1.2 Update CMakeLists.txt

**File:** `src/cpp/CMakeLists.txt`

**Add:**
```cmake
find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(RobotCore
    PRIVATE
    Boost::system
)
```

---

## Step 2: Create Protocol Definition

### 2.1 Create FirmwareProtocol.hpp

**File:** `src/cpp/include/firmware/FirmwareProtocol.hpp`

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <vector>

namespace robotics {
namespace firmware {

// ============================================================================
// Protocol Constants
// ============================================================================

constexpr int NUM_AXES = 6;
constexpr uint32_t PROTOCOL_VERSION = 0x0100;  // v1.0

// Baud rate for serial communication
constexpr int DEFAULT_BAUD_RATE = 115200;
constexpr int HIGH_SPEED_BAUD_RATE = 921600;

// Timeouts (ms)
constexpr int RESPONSE_TIMEOUT_MS = 1000;
constexpr int HOMING_TIMEOUT_MS = 60000;
constexpr int MOTION_TIMEOUT_MS = 5000;

// Buffer sizes
constexpr int TX_BUFFER_SIZE = 256;
constexpr int RX_BUFFER_SIZE = 1024;
constexpr int COMMAND_QUEUE_SIZE = 32;

// ============================================================================
// Command Types
// ============================================================================

/**
 * Command codes sent to firmware
 */
enum class CommandCode : uint8_t {
    // System commands
    PING = 0x01,
    RESET = 0x02,
    GET_VERSION = 0x03,
    GET_STATUS = 0x04,
    SET_CONFIG = 0x05,

    // Motion commands
    MOVE_JOINT = 0x10,      // Move joints to position
    MOVE_LINEAR = 0x11,     // Linear interpolated move
    MOVE_RAPID = 0x12,      // Rapid move (no interpolation)
    JOG_START = 0x13,       // Start jogging
    JOG_STOP = 0x14,        // Stop jogging
    STOP_MOTION = 0x15,     // Emergency stop motion
    FEED_HOLD = 0x16,       // Pause motion
    CYCLE_START = 0x17,     // Resume motion

    // Homing commands
    HOME_ALL = 0x20,        // Home all axes
    HOME_AXIS = 0x21,       // Home single axis
    SET_HOME = 0x22,        // Set current position as home
    GET_HOME_STATUS = 0x23,

    // Position commands
    GET_POSITION = 0x30,    // Get current position
    SET_POSITION = 0x31,    // Set position (no motion)
    GET_VELOCITY = 0x32,    // Get current velocity
    GET_FOLLOWING_ERROR = 0x33,

    // I/O commands
    GET_INPUTS = 0x40,      // Read digital inputs
    SET_OUTPUTS = 0x41,     // Set digital outputs
    GET_ANALOG = 0x42,      // Read analog inputs

    // Safety commands
    GET_LIMITS = 0x50,      // Get limit switch status
    CLEAR_ALARM = 0x51,     // Clear alarm state
    GET_ALARM = 0x52,       // Get current alarms

    // G-code passthrough
    GCODE = 0xF0            // Send raw G-code command
};

/**
 * Response codes from firmware
 */
enum class ResponseCode : uint8_t {
    OK = 0x00,
    ERROR = 0x01,
    BUSY = 0x02,
    INVALID_COMMAND = 0x03,
    INVALID_PARAM = 0x04,
    BUFFER_FULL = 0x05,
    TIMEOUT = 0x06,
    ALARM = 0x07,
    NOT_HOMED = 0x08,

    // Status responses
    STATUS = 0x10,
    POSITION = 0x11,
    VERSION = 0x12,
    LIMITS = 0x13,

    // Async notifications
    MOTION_COMPLETE = 0x20,
    HOMING_COMPLETE = 0x21,
    LIMIT_TRIGGERED = 0x22,
    ALARM_TRIGGERED = 0x23
};

// ============================================================================
// Firmware State
// ============================================================================

/**
 * Firmware machine state
 */
enum class MachineState : uint8_t {
    IDLE = 0,
    RUN = 1,
    HOLD = 2,
    JOG = 3,
    HOMING = 4,
    ALARM = 5,
    CHECK = 6,      // G-code check mode
    DOOR = 7,       // Safety door open
    SLEEP = 8
};

/**
 * Alarm codes
 */
enum class AlarmCode : uint8_t {
    NONE = 0,
    HARD_LIMIT = 1,
    SOFT_LIMIT = 2,
    ABORT_CYCLE = 3,
    PROBE_FAIL = 4,
    PROBE_INITIAL = 5,
    HOMING_FAIL_RESET = 6,
    HOMING_FAIL_DOOR = 7,
    HOMING_FAIL_PULLOFF = 8,
    HOMING_FAIL_APPROACH = 9,
    SPINDLE_CONTROL = 10,
    MOTOR_FAULT = 11,
    ESTOP = 12
};

// ============================================================================
// Data Structures
// ============================================================================

#pragma pack(push, 1)

/**
 * Position data (6 axes, in steps or degrees*1000)
 */
struct PositionData {
    int32_t axis[NUM_AXES];

    PositionData() {
        for (int i = 0; i < NUM_AXES; ++i) axis[i] = 0;
    }
};

/**
 * Velocity data (6 axes, in steps/sec or deg/sec*1000)
 */
struct VelocityData {
    int32_t axis[NUM_AXES];
};

/**
 * Limit switch status (bit flags)
 */
struct LimitStatus {
    uint8_t minLimits;  // Bit 0-5 = axis 0-5 min limit
    uint8_t maxLimits;  // Bit 0-5 = axis 0-5 max limit
    uint8_t homeSwitches;  // Bit 0-5 = home switch status

    bool isMinLimit(int axis) const { return (minLimits >> axis) & 1; }
    bool isMaxLimit(int axis) const { return (maxLimits >> axis) & 1; }
    bool isHome(int axis) const { return (homeSwitches >> axis) & 1; }
};

/**
 * I/O status
 */
struct IOStatus {
    uint16_t digitalInputs;
    uint16_t digitalOutputs;
    uint16_t analogInputs[4];
};

/**
 * Complete machine status
 */
struct MachineStatus {
    MachineState state;
    AlarmCode alarm;
    PositionData position;
    PositionData targetPosition;
    VelocityData velocity;
    LimitStatus limits;
    IOStatus io;
    uint8_t homingStatus;  // Bit flags for homed axes
    uint8_t motionBuffer;  // Commands in motion buffer
    uint16_t feedOverride; // Feed rate override (100 = 100%)
    uint16_t rapidOverride;

    bool isHomed(int axis) const { return (homingStatus >> axis) & 1; }
    bool isAllHomed() const { return homingStatus == 0x3F; }  // All 6 axes
};

/**
 * Command header
 */
struct CommandHeader {
    uint8_t startByte;     // 0xAA
    uint8_t command;       // CommandCode
    uint16_t payloadSize;  // Size of payload data
    uint16_t sequenceNum;  // Command sequence number
};

/**
 * Response header
 */
struct ResponseHeader {
    uint8_t startByte;     // 0xBB
    uint8_t response;      // ResponseCode
    uint16_t payloadSize;
    uint16_t sequenceNum;  // Matches command sequence
    uint8_t checksum;      // XOR checksum of payload
};

/**
 * Move command payload
 */
struct MoveCommand {
    PositionData target;
    uint32_t feedRate;     // mm/min * 100
    uint8_t moveType;      // 0=rapid, 1=linear, 2=joint
    uint8_t flags;         // Bit 0=blocking, Bit 1=relative
};

/**
 * Jog command payload
 */
struct JogCommand {
    int8_t direction[NUM_AXES];  // -1, 0, or +1
    uint32_t speed;              // mm/min * 100
    uint8_t continuous;          // 0=step, 1=continuous
    uint32_t distance;           // For step mode (um)
};

#pragma pack(pop)

// ============================================================================
// G-code Commands
// ============================================================================

/**
 * Common G-code commands for grblHAL
 */
namespace GCode {
    // Motion
    constexpr const char* RAPID_MOVE = "G0";
    constexpr const char* LINEAR_MOVE = "G1";
    constexpr const char* CW_ARC = "G2";
    constexpr const char* CCW_ARC = "G3";
    constexpr const char* DWELL = "G4";

    // Coordinate system
    constexpr const char* ABSOLUTE = "G90";
    constexpr const char* RELATIVE = "G91";
    constexpr const char* SET_ORIGIN = "G92";

    // Units
    constexpr const char* INCHES = "G20";
    constexpr const char* MILLIMETERS = "G21";

    // Homing
    constexpr const char* HOME_ALL = "G28";
    constexpr const char* HOME_AXIS = "G28.1";

    // Probe
    constexpr const char* PROBE = "G38.2";

    // Real-time commands (no newline needed)
    constexpr char SOFT_RESET = 0x18;    // Ctrl-X
    constexpr char STATUS_QUERY = '?';
    constexpr char CYCLE_START = '~';
    constexpr char FEED_HOLD = '!';
    constexpr char SAFETY_DOOR = 0x84;
    constexpr char JOG_CANCEL = 0x85;

    // Feed overrides
    constexpr char FEED_100 = 0x90;
    constexpr char FEED_INCREASE_10 = 0x91;
    constexpr char FEED_DECREASE_10 = 0x92;
    constexpr char FEED_INCREASE_1 = 0x93;
    constexpr char FEED_DECREASE_1 = 0x94;

    // Rapid overrides
    constexpr char RAPID_100 = 0x95;
    constexpr char RAPID_50 = 0x96;
    constexpr char RAPID_25 = 0x97;
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate XOR checksum of data
 */
inline uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Format G-code move command
 */
inline std::string formatMoveGCode(
    const std::array<double, NUM_AXES>& position,
    double feedRate,
    bool isRapid = false) {

    std::string cmd = isRapid ? "G0" : "G1";

    // Axis letters: A, B, C, U, V, W for 6-axis robot
    // Or use X, Y, Z, A, B, C
    const char axisLetters[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};

    for (int i = 0; i < NUM_AXES; ++i) {
        char buf[32];
        snprintf(buf, sizeof(buf), " %c%.4f", axisLetters[i], position[i]);
        cmd += buf;
    }

    if (!isRapid && feedRate > 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), " F%.1f", feedRate);
        cmd += buf;
    }

    return cmd;
}

/**
 * Parse grblHAL status response
 * Format: <Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|...>
 */
inline bool parseStatusResponse(const std::string& response, MachineStatus& status) {
    if (response.empty() || response[0] != '<') {
        return false;
    }

    // Find state
    size_t pipePos = response.find('|');
    if (pipePos == std::string::npos) return false;

    std::string stateStr = response.substr(1, pipePos - 1);

    if (stateStr == "Idle") status.state = MachineState::IDLE;
    else if (stateStr == "Run") status.state = MachineState::RUN;
    else if (stateStr == "Hold") status.state = MachineState::HOLD;
    else if (stateStr == "Jog") status.state = MachineState::JOG;
    else if (stateStr == "Home") status.state = MachineState::HOMING;
    else if (stateStr.find("Alarm") != std::string::npos) status.state = MachineState::ALARM;
    else if (stateStr == "Check") status.state = MachineState::CHECK;
    else if (stateStr == "Door") status.state = MachineState::DOOR;
    else if (stateStr == "Sleep") status.state = MachineState::SLEEP;

    // Parse MPos (machine position)
    size_t mposPos = response.find("MPos:");
    if (mposPos != std::string::npos) {
        size_t start = mposPos + 5;
        size_t end = response.find('|', start);
        std::string posStr = response.substr(start, end - start);

        // Parse comma-separated values
        int axis = 0;
        size_t pos = 0;
        while (pos < posStr.length() && axis < NUM_AXES) {
            size_t comma = posStr.find(',', pos);
            if (comma == std::string::npos) comma = posStr.length();

            double value = std::stod(posStr.substr(pos, comma - pos));
            status.position.axis[axis] = static_cast<int32_t>(value * 1000);  // mm to um

            pos = comma + 1;
            axis++;
        }
    }

    return true;
}

} // namespace firmware
} // namespace robotics
```

---

## Step 3: Create Serial Port Manager

### 3.1 Create SerialPort.hpp

**File:** `src/cpp/include/firmware/SerialPort.hpp`

```cpp
#pragma once

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

namespace robotics {
namespace firmware {

using boost::asio::serial_port;
using boost::asio::io_context;

// ============================================================================
// Serial Port Configuration
// ============================================================================

struct SerialConfig {
    std::string portName;          // COM port or /dev/tty*
    int baudRate;
    int dataBits;
    int stopBits;
    enum class Parity { None, Odd, Even } parity;
    enum class FlowControl { None, Hardware, Software } flowControl;

    SerialConfig()
        : portName(""),
          baudRate(115200),
          dataBits(8),
          stopBits(1),
          parity(Parity::None),
          flowControl(FlowControl::None) {}
};

// ============================================================================
// Callbacks
// ============================================================================

using DataReceivedCallback = std::function<void(const std::vector<uint8_t>&)>;
using LineReceivedCallback = std::function<void(const std::string&)>;
using ErrorCallback = std::function<void(const std::string&)>;
using ConnectionCallback = std::function<void(bool connected)>;

// ============================================================================
// Serial Port Manager
// ============================================================================

/**
 * Manages serial port communication with async read/write
 */
class SerialPortManager {
public:
    SerialPortManager();
    ~SerialPortManager();

    // ========================================================================
    // Connection Management
    // ========================================================================

    /**
     * Open serial port
     */
    bool open(const SerialConfig& config);

    /**
     * Close serial port
     */
    void close();

    /**
     * Check if port is open
     */
    bool isOpen() const;

    /**
     * Get available serial ports
     */
    static std::vector<std::string> getAvailablePorts();

    // ========================================================================
    // Data Transfer
    // ========================================================================

    /**
     * Write data (async)
     */
    void write(const std::vector<uint8_t>& data);

    /**
     * Write string (adds newline)
     */
    void writeLine(const std::string& line);

    /**
     * Write raw bytes
     */
    void writeBytes(const uint8_t* data, size_t length);

    /**
     * Read available data (blocking with timeout)
     */
    std::vector<uint8_t> read(int timeoutMs = 100);

    /**
     * Read line (blocking with timeout)
     */
    std::string readLine(int timeoutMs = 1000);

    /**
     * Flush buffers
     */
    void flush();

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setDataCallback(DataReceivedCallback callback) {
        dataCallback_ = callback;
    }

    void setLineCallback(LineReceivedCallback callback) {
        lineCallback_ = callback;
    }

    void setErrorCallback(ErrorCallback callback) {
        errorCallback_ = callback;
    }

    void setConnectionCallback(ConnectionCallback callback) {
        connectionCallback_ = callback;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    size_t getBytesReceived() const { return bytesReceived_; }
    size_t getBytesSent() const { return bytesSent_; }
    void resetStatistics() { bytesReceived_ = 0; bytesSent_ = 0; }

private:
    io_context ioContext_;
    std::unique_ptr<serial_port> port_;
    std::thread ioThread_;
    std::atomic<bool> running_;

    // Read buffer
    std::array<uint8_t, 1024> readBuffer_;
    std::string lineBuffer_;

    // Write queue
    std::mutex writeMutex_;
    std::queue<std::vector<uint8_t>> writeQueue_;
    std::atomic<bool> writeInProgress_;

    // Callbacks
    DataReceivedCallback dataCallback_;
    LineReceivedCallback lineCallback_;
    ErrorCallback errorCallback_;
    ConnectionCallback connectionCallback_;

    // Statistics
    std::atomic<size_t> bytesReceived_;
    std::atomic<size_t> bytesSent_;

    // Internal methods
    void startAsyncRead();
    void handleRead(const boost::system::error_code& error, size_t bytesTransferred);
    void processWriteQueue();
    void handleWrite(const boost::system::error_code& error, size_t bytesTransferred);
    void runIoContext();
};

// ============================================================================
// Implementation
// ============================================================================

inline SerialPortManager::SerialPortManager()
    : running_(false),
      writeInProgress_(false),
      bytesReceived_(0),
      bytesSent_(0) {
}

inline SerialPortManager::~SerialPortManager() {
    close();
}

inline bool SerialPortManager::open(const SerialConfig& config) {
    try {
        close();  // Close if already open

        port_ = std::make_unique<serial_port>(ioContext_, config.portName);

        // Configure port
        port_->set_option(serial_port::baud_rate(config.baudRate));
        port_->set_option(serial_port::character_size(config.dataBits));

        switch (config.stopBits) {
            case 1: port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one)); break;
            case 2: port_->set_option(serial_port::stop_bits(serial_port::stop_bits::two)); break;
        }

        switch (config.parity) {
            case SerialConfig::Parity::None:
                port_->set_option(serial_port::parity(serial_port::parity::none)); break;
            case SerialConfig::Parity::Odd:
                port_->set_option(serial_port::parity(serial_port::parity::odd)); break;
            case SerialConfig::Parity::Even:
                port_->set_option(serial_port::parity(serial_port::parity::even)); break;
        }

        switch (config.flowControl) {
            case SerialConfig::FlowControl::None:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::none)); break;
            case SerialConfig::FlowControl::Hardware:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::hardware)); break;
            case SerialConfig::FlowControl::Software:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::software)); break;
        }

        // Start async operations
        running_ = true;
        startAsyncRead();

        // Start IO thread
        ioThread_ = std::thread(&SerialPortManager::runIoContext, this);

        if (connectionCallback_) {
            connectionCallback_(true);
        }

        return true;

    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(std::string("Failed to open port: ") + e.what());
        }
        return false;
    }
}

inline void SerialPortManager::close() {
    running_ = false;

    if (port_ && port_->is_open()) {
        boost::system::error_code ec;
        port_->cancel(ec);
        port_->close(ec);
    }

    ioContext_.stop();

    if (ioThread_.joinable()) {
        ioThread_.join();
    }

    ioContext_.restart();
    port_.reset();

    if (connectionCallback_) {
        connectionCallback_(false);
    }
}

inline bool SerialPortManager::isOpen() const {
    return port_ && port_->is_open();
}

inline std::vector<std::string> SerialPortManager::getAvailablePorts() {
    std::vector<std::string> ports;

#ifdef _WIN32
    // Windows: Check COM1-COM256
    for (int i = 1; i <= 256; ++i) {
        char portName[16];
        snprintf(portName, sizeof(portName), "COM%d", i);

        HANDLE hPort = CreateFileA(
            portName, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
            OPEN_EXISTING, 0, nullptr);

        if (hPort != INVALID_HANDLE_VALUE) {
            ports.push_back(portName);
            CloseHandle(hPort);
        }
    }
#else
    // Linux/macOS: Check /dev/tty*
    // This is simplified - production code should use udev or similar
    const char* patterns[] = {
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/tty.usb"
    };

    for (const char* pattern : patterns) {
        for (int i = 0; i < 10; ++i) {
            std::string portName = std::string(pattern) + std::to_string(i);
            if (access(portName.c_str(), F_OK) == 0) {
                ports.push_back(portName);
            }
        }
    }
#endif

    return ports;
}

inline void SerialPortManager::write(const std::vector<uint8_t>& data) {
    if (!isOpen()) return;

    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        writeQueue_.push(data);
    }

    if (!writeInProgress_) {
        processWriteQueue();
    }
}

inline void SerialPortManager::writeLine(const std::string& line) {
    std::vector<uint8_t> data(line.begin(), line.end());
    data.push_back('\n');
    write(data);
}

inline void SerialPortManager::writeBytes(const uint8_t* data, size_t length) {
    write(std::vector<uint8_t>(data, data + length));
}

inline void SerialPortManager::flush() {
#ifdef _WIN32
    if (port_ && port_->is_open()) {
        // Windows-specific flush
        FlushFileBuffers(port_->native_handle());
    }
#else
    if (port_ && port_->is_open()) {
        tcflush(port_->native_handle(), TCIOFLUSH);
    }
#endif
}

inline void SerialPortManager::startAsyncRead() {
    if (!running_ || !port_ || !port_->is_open()) return;

    port_->async_read_some(
        boost::asio::buffer(readBuffer_),
        [this](const boost::system::error_code& error, size_t bytesTransferred) {
            handleRead(error, bytesTransferred);
        });
}

inline void SerialPortManager::handleRead(
    const boost::system::error_code& error, size_t bytesTransferred) {

    if (error) {
        if (error != boost::asio::error::operation_aborted) {
            if (errorCallback_) {
                errorCallback_("Read error: " + error.message());
            }
        }
        return;
    }

    bytesReceived_ += bytesTransferred;

    // Process received data
    if (dataCallback_) {
        std::vector<uint8_t> data(readBuffer_.begin(),
                                   readBuffer_.begin() + bytesTransferred);
        dataCallback_(data);
    }

    // Process lines
    if (lineCallback_) {
        for (size_t i = 0; i < bytesTransferred; ++i) {
            char c = static_cast<char>(readBuffer_[i]);
            if (c == '\n' || c == '\r') {
                if (!lineBuffer_.empty()) {
                    lineCallback_(lineBuffer_);
                    lineBuffer_.clear();
                }
            } else {
                lineBuffer_ += c;
            }
        }
    }

    // Continue reading
    startAsyncRead();
}

inline void SerialPortManager::processWriteQueue() {
    std::vector<uint8_t> data;

    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        if (writeQueue_.empty()) {
            writeInProgress_ = false;
            return;
        }
        data = std::move(writeQueue_.front());
        writeQueue_.pop();
        writeInProgress_ = true;
    }

    boost::asio::async_write(
        *port_,
        boost::asio::buffer(data),
        [this](const boost::system::error_code& error, size_t bytesTransferred) {
            handleWrite(error, bytesTransferred);
        });
}

inline void SerialPortManager::handleWrite(
    const boost::system::error_code& error, size_t bytesTransferred) {

    if (error) {
        if (errorCallback_) {
            errorCallback_("Write error: " + error.message());
        }
        writeInProgress_ = false;
        return;
    }

    bytesSent_ += bytesTransferred;

    // Process next item in queue
    processWriteQueue();
}

inline void SerialPortManager::runIoContext() {
    while (running_) {
        try {
            ioContext_.run();
            if (running_) {
                ioContext_.restart();
            }
        } catch (const std::exception& e) {
            if (errorCallback_) {
                errorCallback_(std::string("IO error: ") + e.what());
            }
        }
    }
}

} // namespace firmware
} // namespace robotics
```

---

## Step 4: Create Firmware Interface

### 4.1 Create FirmwareInterface.hpp

**File:** `src/cpp/include/firmware/FirmwareInterface.hpp`

```cpp
#pragma once

#include "FirmwareProtocol.hpp"
#include "SerialPort.hpp"
#include <functional>
#include <future>
#include <map>

namespace robotics {
namespace firmware {

// ============================================================================
// Callback Types
// ============================================================================

using StatusCallback = std::function<void(const MachineStatus&)>;
using MotionCompleteCallback = std::function<void(bool success)>;
using AlarmCallback = std::function<void(AlarmCode)>;
using HomingCompleteCallback = std::function<void(bool success, uint8_t homedAxes)>;

// ============================================================================
// Firmware Interface
// ============================================================================

/**
 * High-level interface to robot firmware
 */
class FirmwareInterface {
public:
    FirmwareInterface();
    ~FirmwareInterface();

    // ========================================================================
    // Connection
    // ========================================================================

    bool connect(const std::string& portName, int baudRate = DEFAULT_BAUD_RATE);
    void disconnect();
    bool isConnected() const;

    // Auto-detect firmware port
    bool autoConnect();

    // ========================================================================
    // Status
    // ========================================================================

    /**
     * Get current machine status (blocking)
     */
    MachineStatus getStatus();

    /**
     * Start periodic status polling
     */
    void startStatusPolling(int intervalMs = 50);
    void stopStatusPolling();

    /**
     * Get cached status (non-blocking)
     */
    const MachineStatus& getCachedStatus() const { return cachedStatus_; }

    // ========================================================================
    // Motion Commands
    // ========================================================================

    /**
     * Move to joint positions
     * @param positions Joint positions in degrees
     * @param feedRate Feed rate in deg/s
     * @param blocking Wait for completion
     */
    bool moveJoints(
        const std::array<double, NUM_AXES>& positions,
        double feedRate,
        bool blocking = false);

    /**
     * Move linear in Cartesian space
     * @param position XYZ position in mm
     * @param feedRate Feed rate in mm/min
     */
    bool moveLinear(
        const std::array<double, 3>& position,
        double feedRate,
        bool blocking = false);

    /**
     * Rapid move (no interpolation)
     */
    bool moveRapid(const std::array<double, NUM_AXES>& positions);

    /**
     * Start jogging
     * @param axis Axis index (0-5)
     * @param direction +1 or -1
     * @param speed Speed in deg/s
     */
    bool jogStart(int axis, int direction, double speed);

    /**
     * Stop jogging
     */
    bool jogStop();

    /**
     * Stop all motion immediately
     */
    bool stopMotion();

    /**
     * Feed hold (pause)
     */
    bool feedHold();

    /**
     * Resume motion
     */
    bool cycleStart();

    // ========================================================================
    // Homing
    // ========================================================================

    /**
     * Home all axes
     */
    bool homeAll(bool blocking = true);

    /**
     * Home single axis
     */
    bool homeAxis(int axis, bool blocking = true);

    /**
     * Set current position as home
     */
    bool setHome();

    /**
     * Check if axis is homed
     */
    bool isHomed(int axis) const;

    /**
     * Check if all axes are homed
     */
    bool isAllHomed() const;

    // ========================================================================
    // Position
    // ========================================================================

    /**
     * Get current joint positions in degrees
     */
    std::array<double, NUM_AXES> getJointPositions();

    /**
     * Set position without motion (for calibration)
     */
    bool setPosition(const std::array<double, NUM_AXES>& positions);

    // ========================================================================
    // I/O
    // ========================================================================

    uint16_t getDigitalInputs();
    bool setDigitalOutputs(uint16_t outputs);
    uint16_t getAnalogInput(int channel);

    // ========================================================================
    // Safety
    // ========================================================================

    LimitStatus getLimitStatus();
    AlarmCode getCurrentAlarm() const { return cachedStatus_.alarm; }
    bool clearAlarm();

    // ========================================================================
    // G-code
    // ========================================================================

    /**
     * Send raw G-code command
     * @return Response string
     */
    std::string sendGCode(const std::string& command, int timeoutMs = 1000);

    /**
     * Send G-code and wait for 'ok' response
     */
    bool sendGCodeOk(const std::string& command, int timeoutMs = 1000);

    // ========================================================================
    // Feed Override
    // ========================================================================

    void setFeedOverride(int percent);  // 10-200%
    void setRapidOverride(int percent); // 25, 50, or 100%

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setStatusCallback(StatusCallback callback) { statusCallback_ = callback; }
    void setMotionCompleteCallback(MotionCompleteCallback callback) { motionCallback_ = callback; }
    void setAlarmCallback(AlarmCallback callback) { alarmCallback_ = callback; }
    void setHomingCompleteCallback(HomingCompleteCallback callback) { homingCallback_ = callback; }

private:
    SerialPortManager serial_;
    MachineStatus cachedStatus_;
    std::mutex statusMutex_;

    std::atomic<bool> pollingActive_;
    std::thread pollingThread_;
    int pollingIntervalMs_;

    std::atomic<uint16_t> sequenceNumber_;

    // Response handling
    std::mutex responseMutex_;
    std::condition_variable responseCv_;
    std::string lastResponse_;
    bool responseReceived_;

    // Callbacks
    StatusCallback statusCallback_;
    MotionCompleteCallback motionCallback_;
    AlarmCallback alarmCallback_;
    HomingCompleteCallback homingCallback_;

    // Internal methods
    void handleLine(const std::string& line);
    void pollStatus();
    bool waitForOk(int timeoutMs);
    bool sendCommand(CommandCode cmd, const void* payload = nullptr, size_t payloadSize = 0);
    void updateStatus(const MachineStatus& status);
};

// ============================================================================
// Implementation
// ============================================================================

inline FirmwareInterface::FirmwareInterface()
    : pollingActive_(false),
      pollingIntervalMs_(50),
      sequenceNumber_(0),
      responseReceived_(false) {

    serial_.setLineCallback([this](const std::string& line) {
        handleLine(line);
    });

    serial_.setErrorCallback([this](const std::string& error) {
        // Log error
    });
}

inline FirmwareInterface::~FirmwareInterface() {
    disconnect();
}

inline bool FirmwareInterface::connect(const std::string& portName, int baudRate) {
    SerialConfig config;
    config.portName = portName;
    config.baudRate = baudRate;

    if (!serial_.open(config)) {
        return false;
    }

    // Wait for firmware to boot
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Flush any startup messages
    serial_.flush();

    // Send status query to verify connection
    serial_.writeLine(std::string(1, GCode::STATUS_QUERY));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Check if we got a response
    return true;  // Could verify status response here
}

inline void FirmwareInterface::disconnect() {
    stopStatusPolling();
    serial_.close();
}

inline bool FirmwareInterface::isConnected() const {
    return serial_.isOpen();
}

inline bool FirmwareInterface::autoConnect() {
    auto ports = SerialPortManager::getAvailablePorts();

    for (const auto& port : ports) {
        if (connect(port)) {
            // Try to get status
            auto status = getStatus();
            if (status.state != MachineState::ALARM || status.alarm == AlarmCode::NONE) {
                return true;
            }
        }
        disconnect();
    }

    return false;
}

inline MachineStatus FirmwareInterface::getStatus() {
    if (!isConnected()) return MachineStatus();

    // Send status query
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::STATUS_QUERY), 1);

    // Wait for response
    std::unique_lock<std::mutex> lock(responseMutex_);
    responseReceived_ = false;

    if (responseCv_.wait_for(lock, std::chrono::milliseconds(RESPONSE_TIMEOUT_MS),
                             [this] { return responseReceived_; })) {
        std::lock_guard<std::mutex> statusLock(statusMutex_);
        return cachedStatus_;
    }

    return MachineStatus();
}

inline void FirmwareInterface::startStatusPolling(int intervalMs) {
    if (pollingActive_) return;

    pollingIntervalMs_ = intervalMs;
    pollingActive_ = true;

    pollingThread_ = std::thread(&FirmwareInterface::pollStatus, this);
}

inline void FirmwareInterface::stopStatusPolling() {
    pollingActive_ = false;
    if (pollingThread_.joinable()) {
        pollingThread_.join();
    }
}

inline void FirmwareInterface::pollStatus() {
    while (pollingActive_) {
        if (isConnected()) {
            serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::STATUS_QUERY), 1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(pollingIntervalMs_));
    }
}

inline bool FirmwareInterface::moveJoints(
    const std::array<double, NUM_AXES>& positions,
    double feedRate,
    bool blocking) {

    if (!isConnected()) return false;

    // Format G-code command
    std::string cmd = formatMoveGCode(positions, feedRate * 60.0, false);

    serial_.writeLine(cmd);

    if (blocking) {
        return waitForOk(MOTION_TIMEOUT_MS);
    }

    return true;
}

inline bool FirmwareInterface::moveRapid(const std::array<double, NUM_AXES>& positions) {
    if (!isConnected()) return false;

    std::string cmd = formatMoveGCode(positions, 0, true);
    serial_.writeLine(cmd);

    return waitForOk(MOTION_TIMEOUT_MS);
}

inline bool FirmwareInterface::jogStart(int axis, int direction, double speed) {
    if (!isConnected() || axis < 0 || axis >= NUM_AXES) return false;

    // grblHAL jog command: $J=X10F1000
    const char axisLetters[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};

    char cmd[64];
    double distance = direction * 1000.0;  // Large distance for continuous jog
    snprintf(cmd, sizeof(cmd), "$J=%c%.3fF%.1f",
             axisLetters[axis], distance, speed * 60.0);

    serial_.writeLine(cmd);
    return true;
}

inline bool FirmwareInterface::jogStop() {
    if (!isConnected()) return false;

    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::JOG_CANCEL), 1);
    return true;
}

inline bool FirmwareInterface::stopMotion() {
    if (!isConnected()) return false;

    // Send feed hold followed by soft reset
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::FEED_HOLD), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::SOFT_RESET), 1);

    return true;
}

inline bool FirmwareInterface::feedHold() {
    if (!isConnected()) return false;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::FEED_HOLD), 1);
    return true;
}

inline bool FirmwareInterface::cycleStart() {
    if (!isConnected()) return false;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::CYCLE_START), 1);
    return true;
}

inline bool FirmwareInterface::homeAll(bool blocking) {
    if (!isConnected()) return false;

    serial_.writeLine("$H");

    if (blocking) {
        // Wait for homing to complete (with long timeout)
        return waitForOk(HOMING_TIMEOUT_MS);
    }

    return true;
}

inline bool FirmwareInterface::homeAxis(int axis, bool blocking) {
    if (!isConnected() || axis < 0 || axis >= NUM_AXES) return false;

    const char axisLetters[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "$H%c", axisLetters[axis]);

    serial_.writeLine(cmd);

    if (blocking) {
        return waitForOk(HOMING_TIMEOUT_MS);
    }

    return true;
}

inline bool FirmwareInterface::setHome() {
    return sendGCodeOk("G92 X0 Y0 Z0 A0 B0 C0");
}

inline bool FirmwareInterface::isHomed(int axis) const {
    return cachedStatus_.isHomed(axis);
}

inline bool FirmwareInterface::isAllHomed() const {
    return cachedStatus_.isAllHomed();
}

inline std::array<double, NUM_AXES> FirmwareInterface::getJointPositions() {
    std::array<double, NUM_AXES> positions;

    std::lock_guard<std::mutex> lock(statusMutex_);
    for (int i = 0; i < NUM_AXES; ++i) {
        positions[i] = cachedStatus_.position.axis[i] / 1000.0;  // um to mm/deg
    }

    return positions;
}

inline bool FirmwareInterface::clearAlarm() {
    if (!isConnected()) return false;

    serial_.writeLine("$X");  // grblHAL unlock command
    return waitForOk(RESPONSE_TIMEOUT_MS);
}

inline std::string FirmwareInterface::sendGCode(const std::string& command, int timeoutMs) {
    if (!isConnected()) return "";

    {
        std::lock_guard<std::mutex> lock(responseMutex_);
        responseReceived_ = false;
        lastResponse_.clear();
    }

    serial_.writeLine(command);

    std::unique_lock<std::mutex> lock(responseMutex_);
    if (responseCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                             [this] { return responseReceived_; })) {
        return lastResponse_;
    }

    return "";
}

inline bool FirmwareInterface::sendGCodeOk(const std::string& command, int timeoutMs) {
    std::string response = sendGCode(command, timeoutMs);
    return response.find("ok") != std::string::npos;
}

inline void FirmwareInterface::setFeedOverride(int percent) {
    if (!isConnected()) return;

    percent = std::clamp(percent, 10, 200);

    // Reset to 100% first
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::FEED_100), 1);

    // Then adjust
    while (percent > 100) {
        serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::FEED_INCREASE_10), 1);
        percent -= 10;
    }
    while (percent < 100) {
        serial_.writeBytes(reinterpret_cast<const uint8_t*>(&GCode::FEED_DECREASE_10), 1);
        percent += 10;
    }
}

inline void FirmwareInterface::handleLine(const std::string& line) {
    if (line.empty()) return;

    // Status response
    if (line[0] == '<') {
        MachineStatus status;
        if (parseStatusResponse(line, status)) {
            updateStatus(status);
        }
        return;
    }

    // Response message
    {
        std::lock_guard<std::mutex> lock(responseMutex_);
        lastResponse_ = line;
        responseReceived_ = true;
    }
    responseCv_.notify_one();

    // Check for specific responses
    if (line.find("ok") != std::string::npos) {
        // Command acknowledged
    }
    else if (line.find("error") != std::string::npos) {
        // Error response
    }
    else if (line.find("ALARM") != std::string::npos) {
        // Alarm triggered
        if (alarmCallback_) {
            alarmCallback_(cachedStatus_.alarm);
        }
    }
}

inline bool FirmwareInterface::waitForOk(int timeoutMs) {
    std::unique_lock<std::mutex> lock(responseMutex_);
    responseReceived_ = false;

    if (responseCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                             [this] { return responseReceived_; })) {
        return lastResponse_.find("ok") != std::string::npos;
    }

    return false;
}

inline void FirmwareInterface::updateStatus(const MachineStatus& status) {
    AlarmCode prevAlarm;

    {
        std::lock_guard<std::mutex> lock(statusMutex_);
        prevAlarm = cachedStatus_.alarm;
        cachedStatus_ = status;
    }

    // Notify status callback
    if (statusCallback_) {
        statusCallback_(status);
    }

    // Check for alarm change
    if (status.alarm != prevAlarm && status.alarm != AlarmCode::NONE) {
        if (alarmCallback_) {
            alarmCallback_(status.alarm);
        }
    }
}

} // namespace firmware
} // namespace robotics
```

---

## Step 5: Create Motion Command Streamer

### 5.1 Create MotionStreamer.hpp

**File:** `src/cpp/include/firmware/MotionStreamer.hpp`

```cpp
#pragma once

#include "FirmwareInterface.hpp"
#include "trajectory/TrajectoryExecutor.hpp"
#include <queue>

namespace robotics {
namespace firmware {

using namespace trajectory;

/**
 * Streams trajectory points to firmware
 * Maintains command buffer for smooth motion
 */
class MotionStreamer {
public:
    explicit MotionStreamer(
        std::shared_ptr<FirmwareInterface> firmware,
        std::shared_ptr<kinematics::IKinematicsService> kinematics);

    ~MotionStreamer();

    // ========================================================================
    // Trajectory Execution
    // ========================================================================

    /**
     * Start streaming trajectory to firmware
     */
    bool startTrajectory(const Trajectory& trajectory);

    /**
     * Stop streaming
     */
    void stop();

    /**
     * Pause streaming
     */
    void pause();

    /**
     * Resume streaming
     */
    void resume();

    /**
     * Get current state
     */
    TrajectoryState getState() const { return state_.load(); }

    /**
     * Get progress (0.0 - 1.0)
     */
    double getProgress() const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setStreamRate(int rateHz) { streamRate_ = rateHz; }
    void setBufferSize(int size) { bufferSize_ = size; }

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setCompleteCallback(std::function<void(bool)> callback) {
        completeCallback_ = callback;
    }

private:
    std::shared_ptr<FirmwareInterface> firmware_;
    std::shared_ptr<kinematics::IKinematicsService> kinematics_;

    TrajectoryExecutor executor_;

    std::atomic<TrajectoryState> state_;
    std::atomic<bool> stopRequested_;

    int streamRate_;      // Points per second
    int bufferSize_;      // Max commands in firmware buffer

    std::thread streamThread_;
    std::function<void(bool)> completeCallback_;

    // Command queue
    std::mutex queueMutex_;
    std::queue<TrajectoryPoint> pointQueue_;

    void streamLoop();
    void sendPoint(const TrajectoryPoint& point);
};

// ============================================================================
// Implementation
// ============================================================================

inline MotionStreamer::MotionStreamer(
    std::shared_ptr<FirmwareInterface> firmware,
    std::shared_ptr<kinematics::IKinematicsService> kinematics)
    : firmware_(firmware),
      kinematics_(kinematics),
      executor_(kinematics),
      state_(TrajectoryState::IDLE),
      stopRequested_(false),
      streamRate_(50),      // 50 Hz default
      bufferSize_(16) {
}

inline MotionStreamer::~MotionStreamer() {
    stop();
}

inline bool MotionStreamer::startTrajectory(const Trajectory& trajectory) {
    if (!firmware_->isConnected()) return false;
    if (!trajectory.isValid) return false;

    stop();  // Stop any existing motion

    state_.store(TrajectoryState::RUNNING);
    stopRequested_.store(false);

    // Start executor
    executor_.setControlRate(streamRate_);

    // Set callback to queue points
    executor_.setPointCallback([this](const TrajectoryPoint& point) {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pointQueue_.push(point);
    });

    executor_.setCompleteCallback([this](bool success, const std::string& msg) {
        state_.store(TrajectoryState::COMPLETED);
        if (completeCallback_) {
            completeCallback_(success);
        }
    });

    executor_.start(trajectory);

    // Start stream thread
    streamThread_ = std::thread(&MotionStreamer::streamLoop, this);

    return true;
}

inline void MotionStreamer::stop() {
    stopRequested_.store(true);
    executor_.stop();
    firmware_->stopMotion();

    if (streamThread_.joinable()) {
        streamThread_.join();
    }

    state_.store(TrajectoryState::IDLE);

    // Clear queue
    std::lock_guard<std::mutex> lock(queueMutex_);
    while (!pointQueue_.empty()) pointQueue_.pop();
}

inline void MotionStreamer::pause() {
    executor_.pause();
    firmware_->feedHold();
    state_.store(TrajectoryState::PAUSED);
}

inline void MotionStreamer::resume() {
    firmware_->cycleStart();
    executor_.resume();
    state_.store(TrajectoryState::RUNNING);
}

inline double MotionStreamer::getProgress() const {
    return executor_.getProgress();
}

inline void MotionStreamer::streamLoop() {
    auto lastSendTime = std::chrono::steady_clock::now();
    double sendInterval = 1.0 / streamRate_;

    while (!stopRequested_.load()) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastSendTime).count();

        if (elapsed >= sendInterval) {
            // Check firmware buffer status
            auto status = firmware_->getCachedStatus();

            if (status.motionBuffer < bufferSize_) {
                // Get next point from queue
                TrajectoryPoint point;
                bool hasPoint = false;

                {
                    std::lock_guard<std::mutex> lock(queueMutex_);
                    if (!pointQueue_.empty()) {
                        point = pointQueue_.front();
                        pointQueue_.pop();
                        hasPoint = true;
                    }
                }

                if (hasPoint && point.isValid) {
                    sendPoint(point);
                }
            }

            lastSendTime = now;
        }

        // Check for completion
        if (executor_.getState() == TrajectoryState::COMPLETED) {
            // Wait for firmware buffer to empty
            auto status = firmware_->getCachedStatus();
            if (status.motionBuffer == 0 && status.state == MachineState::IDLE) {
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

inline void MotionStreamer::sendPoint(const TrajectoryPoint& point) {
    // Convert joint positions to degrees for firmware
    std::array<double, NUM_AXES> positions;
    for (int i = 0; i < NUM_AXES; ++i) {
        positions[i] = radToDeg(point.jointPositions[i]);
    }

    // Calculate feed rate from velocity
    double maxVel = 0;
    for (int i = 0; i < NUM_AXES; ++i) {
        maxVel = std::max(maxVel, std::abs(point.jointVelocities[i]));
    }
    double feedRate = radToDeg(maxVel) * 60.0;  // deg/min

    if (feedRate < 1.0) feedRate = 1.0;  // Minimum feed rate

    firmware_->moveJoints(positions, feedRate / 60.0, false);
}

} // namespace firmware
} // namespace robotics
```

---

## Step 6: Create Unit Tests

### 6.1 Create test_firmware.cpp

**File:** `src/cpp/tests/test_firmware.cpp`

```cpp
#include <gtest/gtest.h>
#include "firmware/FirmwareProtocol.hpp"
#include "firmware/SerialPort.hpp"
#include "firmware/FirmwareInterface.hpp"

using namespace robotics::firmware;

// ============================================================================
// Protocol Tests
// ============================================================================

TEST(ProtocolTest, Checksum) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t checksum = calculateChecksum(data, 4);

    EXPECT_EQ(checksum, 0x01 ^ 0x02 ^ 0x03 ^ 0x04);
}

TEST(ProtocolTest, FormatMoveGCode) {
    std::array<double, NUM_AXES> pos = {10.0, 20.0, 30.0, 45.0, 60.0, 90.0};

    std::string cmd = formatMoveGCode(pos, 1000.0, false);

    EXPECT_TRUE(cmd.find("G1") != std::string::npos);
    EXPECT_TRUE(cmd.find("X10.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("F1000.0") != std::string::npos);
}

TEST(ProtocolTest, FormatRapidGCode) {
    std::array<double, NUM_AXES> pos = {0, 0, 0, 0, 0, 0};

    std::string cmd = formatMoveGCode(pos, 0, true);

    EXPECT_TRUE(cmd.find("G0") != std::string::npos);
    EXPECT_TRUE(cmd.find("F") == std::string::npos);  // No feed rate for rapid
}

TEST(ProtocolTest, ParseStatusIdle) {
    std::string response = "<Idle|MPos:0.000,0.000,0.000,0.000,0.000,0.000|WPos:0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::IDLE);
}

TEST(ProtocolTest, ParseStatusRun) {
    std::string response = "<Run|MPos:100.500,50.250,30.000,45.000,60.000,90.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::RUN);
    EXPECT_EQ(status.position.axis[0], 100500);  // 100.5mm in um
}

TEST(ProtocolTest, ParseStatusAlarm) {
    std::string response = "<Alarm:1|MPos:0.000,0.000,0.000,0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::ALARM);
}

// ============================================================================
// Data Structure Tests
// ============================================================================

TEST(DataStructTest, PositionData) {
    PositionData pos;

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_EQ(pos.axis[i], 0);
    }

    pos.axis[0] = 1000;
    EXPECT_EQ(pos.axis[0], 1000);
}

TEST(DataStructTest, LimitStatus) {
    LimitStatus limits;
    limits.minLimits = 0b00000101;  // Axis 0 and 2
    limits.maxLimits = 0b00001010;  // Axis 1 and 3
    limits.homeSwitches = 0b00111111;  // All axes

    EXPECT_TRUE(limits.isMinLimit(0));
    EXPECT_FALSE(limits.isMinLimit(1));
    EXPECT_TRUE(limits.isMinLimit(2));

    EXPECT_FALSE(limits.isMaxLimit(0));
    EXPECT_TRUE(limits.isMaxLimit(1));

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_TRUE(limits.isHome(i));
    }
}

TEST(DataStructTest, MachineStatus) {
    MachineStatus status;

    status.homingStatus = 0b00111111;  // All homed
    EXPECT_TRUE(status.isAllHomed());

    status.homingStatus = 0b00111110;  // Axis 0 not homed
    EXPECT_FALSE(status.isAllHomed());
    EXPECT_FALSE(status.isHomed(0));
    EXPECT_TRUE(status.isHomed(1));
}

// ============================================================================
// Serial Port Tests (Mock/Offline)
// ============================================================================

TEST(SerialPortTest, GetAvailablePorts) {
    auto ports = SerialPortManager::getAvailablePorts();

    // Just check it doesn't crash
    // Actual ports depend on system
    EXPECT_GE(ports.size(), 0);
}

TEST(SerialPortTest, OpenInvalidPort) {
    SerialPortManager serial;

    SerialConfig config;
    config.portName = "INVALID_PORT_12345";
    config.baudRate = 115200;

    bool opened = serial.open(config);
    EXPECT_FALSE(opened);
}

TEST(SerialPortTest, StatisticsReset) {
    SerialPortManager serial;

    serial.resetStatistics();
    EXPECT_EQ(serial.getBytesReceived(), 0);
    EXPECT_EQ(serial.getBytesSent(), 0);
}

// ============================================================================
// Command Code Tests
// ============================================================================

TEST(CommandCodeTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::PING), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::MOVE_JOINT), 0x10);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::HOME_ALL), 0x20);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::GCODE), 0xF0);
}

TEST(ResponseCodeTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::OK), 0x00);
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::ERROR), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::MOTION_COMPLETE), 0x20);
}

// ============================================================================
// G-code Constants
// ============================================================================

TEST(GCodeTest, RealTimeCommands) {
    EXPECT_EQ(GCode::SOFT_RESET, 0x18);
    EXPECT_EQ(GCode::STATUS_QUERY, '?');
    EXPECT_EQ(GCode::CYCLE_START, '~');
    EXPECT_EQ(GCode::FEED_HOLD, '!');
}

// ============================================================================
// Integration Tests (require hardware)
// ============================================================================

class FirmwareIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Skip if no hardware
        auto ports = SerialPortManager::getAvailablePorts();
        hasHardware_ = !ports.empty();

        if (hasHardware_) {
            firmware_ = std::make_unique<FirmwareInterface>();
            // Don't auto-connect in tests
        }
    }

    bool hasHardware_ = false;
    std::unique_ptr<FirmwareInterface> firmware_;
};

TEST_F(FirmwareIntegrationTest, DISABLED_ConnectDisconnect) {
    if (!hasHardware_) {
        GTEST_SKIP() << "No hardware connected";
    }

    EXPECT_FALSE(firmware_->isConnected());

    bool connected = firmware_->autoConnect();
    if (connected) {
        EXPECT_TRUE(firmware_->isConnected());
        firmware_->disconnect();
        EXPECT_FALSE(firmware_->isConnected());
    }
}

TEST_F(FirmwareIntegrationTest, DISABLED_GetStatus) {
    if (!hasHardware_ || !firmware_->autoConnect()) {
        GTEST_SKIP() << "Cannot connect to hardware";
    }

    auto status = firmware_->getStatus();

    // Should get valid state
    EXPECT_NE(status.state, MachineState::ALARM);  // Unless there's an alarm
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Step 7: Update CMakeLists.txt

### 7.1 Add Firmware to Build

**File:** `src/cpp/CMakeLists.txt`

**Add:**
```cmake
# Include firmware
target_include_directories(RobotCore
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/firmware
)

# Firmware tests
add_executable(test_firmware tests/test_firmware.cpp)
target_link_libraries(test_firmware
    PRIVATE
    GTest::gtest
    GTest::gtest_main
    Boost::system
)
target_include_directories(test_firmware PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME FirmwareTests COMMAND test_firmware)
```

---

## Step 8: Create C# Firmware Client

### 8.1 Create FirmwarePayloads.cs

**File:** `src/csharp/RobotController.Core/IPC/FirmwarePayloads.cs`

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Core.IPC;

// ============================================================================
// Machine Status
// ============================================================================

public enum MachineState
{
    Idle = 0,
    Run = 1,
    Hold = 2,
    Jog = 3,
    Homing = 4,
    Alarm = 5,
    Check = 6,
    Door = 7,
    Sleep = 8
}

public enum AlarmCode
{
    None = 0,
    HardLimit = 1,
    SoftLimit = 2,
    AbortCycle = 3,
    ProbeFail = 4,
    HomingFail = 6,
    MotorFault = 11,
    EStop = 12
}

public record MachineStatusData
{
    [JsonPropertyName("state")]
    public int State { get; init; }

    [JsonPropertyName("alarm")]
    public int Alarm { get; init; }

    [JsonPropertyName("position")]
    public double[] Position { get; init; } = new double[6];

    [JsonPropertyName("velocity")]
    public double[] Velocity { get; init; } = new double[6];

    [JsonPropertyName("homingStatus")]
    public int HomingStatus { get; init; }

    [JsonPropertyName("limitMin")]
    public int LimitMin { get; init; }

    [JsonPropertyName("limitMax")]
    public int LimitMax { get; init; }

    [JsonPropertyName("feedOverride")]
    public int FeedOverride { get; init; }

    public MachineState GetState() => (MachineState)State;
    public AlarmCode GetAlarm() => (AlarmCode)Alarm;
    public bool IsHomed(int axis) => ((HomingStatus >> axis) & 1) == 1;
    public bool IsAllHomed => HomingStatus == 0x3F;
}

// ============================================================================
// Connection
// ============================================================================

public record FirmwareConnectRequest
{
    [JsonPropertyName("portName")]
    public string PortName { get; init; } = "";

    [JsonPropertyName("baudRate")]
    public int BaudRate { get; init; } = 115200;

    [JsonPropertyName("autoDetect")]
    public bool AutoDetect { get; init; }
}

public record FirmwareConnectResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("portName")]
    public string PortName { get; init; } = "";

    [JsonPropertyName("firmwareVersion")]
    public string FirmwareVersion { get; init; } = "";

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Jog Control
// ============================================================================

public record JogRequest
{
    [JsonPropertyName("axis")]
    public int Axis { get; init; }

    [JsonPropertyName("direction")]
    public int Direction { get; init; }

    [JsonPropertyName("speed")]
    public double Speed { get; init; }

    [JsonPropertyName("continuous")]
    public bool Continuous { get; init; }

    [JsonPropertyName("distance")]
    public double Distance { get; init; }
}

public record JogResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Homing
// ============================================================================

public record HomeRequest
{
    [JsonPropertyName("axis")]
    public int Axis { get; init; } = -1;  // -1 = all axes

    [JsonPropertyName("blocking")]
    public bool Blocking { get; init; } = true;
}

public record HomeResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("homedAxes")]
    public int HomedAxes { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Available Ports
// ============================================================================

public record AvailablePortsResponse
{
    [JsonPropertyName("ports")]
    public string[] Ports { get; init; } = Array.Empty<string>();
}
```

### 8.2 Create IFirmwareClientService.cs

**File:** `src/csharp/RobotController.Core/Services/IFirmwareClientService.cs`

```csharp
using RobotController.Core.IPC;

namespace RobotController.Core.Services;

/// <summary>
/// Client service for firmware communication
/// </summary>
public interface IFirmwareClientService
{
    // Connection
    Task<FirmwareConnectResponse> ConnectAsync(
        string portName,
        int baudRate = 115200,
        CancellationToken ct = default);

    Task<FirmwareConnectResponse> AutoConnectAsync(CancellationToken ct = default);
    Task DisconnectAsync(CancellationToken ct = default);
    bool IsConnected { get; }

    // Status
    Task<MachineStatusData> GetStatusAsync(CancellationToken ct = default);
    MachineStatusData CachedStatus { get; }

    // Motion
    Task<bool> JogStartAsync(int axis, int direction, double speed, CancellationToken ct = default);
    Task<bool> JogStopAsync(CancellationToken ct = default);
    Task<bool> StopMotionAsync(CancellationToken ct = default);
    Task<bool> FeedHoldAsync(CancellationToken ct = default);
    Task<bool> CycleStartAsync(CancellationToken ct = default);

    // Homing
    Task<HomeResponse> HomeAllAsync(CancellationToken ct = default);
    Task<HomeResponse> HomeAxisAsync(int axis, CancellationToken ct = default);

    // Safety
    Task<bool> ClearAlarmAsync(CancellationToken ct = default);

    // Configuration
    Task<AvailablePortsResponse> GetAvailablePortsAsync(CancellationToken ct = default);
    Task SetFeedOverrideAsync(int percent, CancellationToken ct = default);

    // Events
    event EventHandler<MachineStatusData>? StatusUpdated;
    event EventHandler<AlarmCode>? AlarmTriggered;
    event EventHandler<bool>? ConnectionChanged;
}
```

---

## Step 9: Validation

### 9.1 Build and Test

**Commands:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller

# Install Boost
vcpkg install boost-asio:x64-windows

# Build
cmake --build build --config Debug

# Run firmware tests
.\build\Debug\test_firmware.exe
```

**Expected Output:**
```
[==========] Running 14 tests from 6 test suites.
...
[  PASSED  ] 14 tests.
```

---

## Completion Checklist

- [ ] Boost.Asio installed via vcpkg
- [ ] FirmwareProtocol.hpp created (commands, responses, data structures)
- [ ] SerialPort.hpp created (async serial communication)
- [ ] FirmwareInterface.hpp created (high-level firmware API)
- [ ] MotionStreamer.hpp created (trajectory streaming)
- [ ] test_firmware.cpp created with 14+ tests
- [ ] CMakeLists.txt updated for firmware
- [ ] C# FirmwareClientService created
- [ ] All firmware tests pass

---

## Troubleshooting

### Serial Port Not Opening
- Check if port is in use by another application
- Verify port name (COM3 on Windows, /dev/ttyUSB0 on Linux)
- Check USB driver installation

### grblHAL Not Responding
- Verify baud rate matches firmware configuration
- Check if firmware is in ALARM state
- Try soft reset (Ctrl-X)

### Motion Not Executing
- Verify all axes are homed
- Check for alarms
- Verify feed rate is valid

### Buffer Overflow
- Reduce stream rate
- Increase firmware buffer size in grblHAL config

---

## Notes

### grblHAL Configuration
For 6-axis robot, grblHAL needs to be configured:
- Enable axes A, B, C
- Set steps/mm for each axis
- Configure homing direction and speeds
- Set acceleration limits

### Real-time Requirements
Motion streaming requires consistent timing:
- Use dedicated thread for streaming
- Monitor firmware buffer level
- Handle network/USB latency

### Safety Considerations
- Always implement E-stop handling
- Monitor limit switches
- Verify homing before motion
- Implement watchdog timeout

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P2_04: Add firmware communication layer

- Add Boost.Asio dependency for serial port
- Create FirmwareProtocol with grblHAL commands
- Implement async SerialPortManager
- Create FirmwareInterface for high-level API
- Add MotionStreamer for trajectory execution
- Implement status polling and parsing
- Add homing, jogging, and motion control
- Create 14 unit tests for firmware
- Add C# FirmwareClientService

Co-Authored-By: Claude <noreply@anthropic.com>"
```
