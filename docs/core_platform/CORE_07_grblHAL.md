# CORE MODULE: grblHAL Driver Interface

## Document Info
| Item | Value |
|------|-------|
| **Module** | grblHAL Driver |
| **Layer** | Core Logic ↔ Firmware Interface |
| **Hardware** | Teensy 4.1 (iMXRT1062) |
| **Firmware** | grblHAL |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Module grblHAL Driver chịu trách nhiệm giao tiếp giữa C++ Core Logic và firmware grblHAL trên Teensy 4.1. Module này biến Teensy thành một "motion card" thông minh, nhận quỹ đạo đã tính toán từ PC và chuyển đổi thành tín hiệu Step/Dir chính xác đến microsecond.

### 1.2. Architecture: Split-Brain Model

```
┌─────────────────────────────────────────────────────────────────┐
│                    SPLIT-BRAIN ARCHITECTURE                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                    PC (The "Cortex")                        │ │
│  │                                                             │ │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │ │
│  │   │  Ruckig     │  │  IK Solver  │  │  Collision  │       │ │
│  │   │  OTG        │  │  (RL/Eigen) │  │  Detection  │       │ │
│  │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘       │ │
│  │          │                │                │               │ │
│  │          └────────────────┴────────────────┘               │ │
│  │                           │                                 │ │
│  │                    Joint Positions                          │ │
│  │                    (θ1...θ6)                                │ │
│  │                           │                                 │ │
│  │                    ┌──────▼──────┐                         │ │
│  │                    │  G-code     │                         │ │
│  │                    │  Generator  │                         │ │
│  │                    └──────┬──────┘                         │ │
│  └───────────────────────────┼─────────────────────────────────┘ │
│                              │                                   │
│                     USB Serial (2Mbaud)                         │
│                     Character Counting Protocol                  │
│                              │                                   │
│  ┌───────────────────────────▼─────────────────────────────────┐ │
│  │              TEENSY 4.1 (The "Brainstem")                   │ │
│  │                                                              │ │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │ │
│  │   │  grblHAL    │  │  Hardware   │  │  Step/Dir   │        │ │
│  │   │  Core       │  │  Timers     │  │  Generator  │        │ │
│  │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │ │
│  │          │                │                │                │ │
│  │          └────────────────┴────────────────┘                │ │
│  │                           │                                  │ │
│  └───────────────────────────┼──────────────────────────────────┘ │
│                              │                                   │
│                    6x Step/Dir Signals                          │
│                              │                                   │
│  ┌───────────────────────────▼─────────────────────────────────┐ │
│  │                    SERVO DRIVERS                             │ │
│  │   ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐          │ │
│  │   │ J1  │ │ J2  │ │ J3  │ │ J4  │ │ J5  │ │ J6  │          │ │
│  │   └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘          │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3. Key Features
- **6-Axis Simultaneous Control**: Điều khiển đồng thời 6 trục robot
- **Planner Bypass**: Vô hiệu hóa planner nội bộ, để Ruckig kiểm soát
- **High-Speed Streaming**: Character Counting protocol cho throughput cao
- **Real-time Feedback**: Báo cáo vị trí và trạng thái bộ đệm
- **Safety I/O**: E-Stop, Limits, và các tín hiệu an toàn

---

## 2. Teensy 4.1 Hardware

### 2.1. iMXRT1062 Specifications

| Feature | Specification |
|---------|---------------|
| **CPU** | ARM Cortex-M7 @ 600 MHz |
| **FPU** | Hardware floating point (single + double) |
| **RAM** | 1 MB (512KB tight-coupled) |
| **Flash** | 8 MB |
| **USB** | High-speed USB 2.0 (480 Mbps) |
| **Timers** | Multiple 32-bit timers for step generation |

### 2.2. Pin Mapping for 6-Axis

```
┌─────────────────────────────────────────────────────────────────┐
│              TEENSY 4.1 PIN MAPPING (6-AXIS ROBOT)               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ╔══════════════════════════════════════════════════════════╗   │
│  ║  STEP/DIR SIGNALS (Primary Motor Control)                ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║                                                           ║   │
│  ║   Axis 1 (X/J1):  Step = Pin 0,   Dir = Pin 1            ║   │
│  ║   Axis 2 (Y/J2):  Step = Pin 2,   Dir = Pin 3            ║   │
│  ║   Axis 3 (Z/J3):  Step = Pin 4,   Dir = Pin 5            ║   │
│  ║   Axis 4 (A/J4):  Step = Pin 6,   Dir = Pin 7            ║   │
│  ║   Axis 5 (B/J5):  Step = Pin 8,   Dir = Pin 9            ║   │
│  ║   Axis 6 (C/J6):  Step = Pin 24,  Dir = Pin 25           ║   │
│  ║                                                           ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║  LIMIT SWITCHES (Homing & Safety)                        ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║                                                           ║   │
│  ║   X Limit: Pin 20        A Limit: Pin 36                 ║   │
│  ║   Y Limit: Pin 21        B Limit: Pin 37                 ║   │
│  ║   Z Limit: Pin 22        C Limit: Pin 38                 ║   │
│  ║                                                           ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║  SAFETY & CONTROL                                        ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║                                                           ║   │
│  ║   E-Stop Input:     Pin 19 (Hardware interrupt)          ║   │
│  ║   Servo Enable:     Pin 10 (Active HIGH)                 ║   │
│  ║   Spindle/Arc On:   Pin 12 (M3/M5 control)               ║   │
│  ║   Spindle PWM:      Pin 13 (0-10V via DAC)               ║   │
│  ║   Coolant/Gas:      Pin 14 (M7/M8/M9)                    ║   │
│  ║   Probe Input:      Pin 23 (G38.2 touch sensing)         ║   │
│  ║                                                           ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║  RESERVED (Ethernet if used)                             ║   │
│  ╠══════════════════════════════════════════════════════════╣   │
│  ║                                                           ║   │
│  ║   If using Ethernet: Pins 34, 35, 41 reserved            ║   │
│  ║                                                           ║   │
│  ╚══════════════════════════════════════════════════════════╝   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Planner Bypass Strategy

### 3.1. Problem Statement

```
┌─────────────────────────────────────────────────────────────────┐
│                    THE PLANNER CONFLICT                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Without Bypass:                                                 │
│  ───────────────                                                │
│                                                                  │
│  Ruckig (PC)                    grblHAL (Teensy)                │
│  ┌─────────────┐                ┌─────────────┐                 │
│  │  S-Curve    │ ─── G-code ──► │ Trapezoidal │                 │
│  │  Planner    │                │  Planner    │                 │
│  └─────────────┘                └─────────────┘                 │
│                                                                  │
│  Two planners = CONFLICT!                                        │
│  • Double acceleration limiting                                  │
│  • Trajectory distortion                                         │
│  • Jerky motion at segment junctions                            │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  With Bypass (Solution):                                         │
│  ───────────────────────                                        │
│                                                                  │
│  Ruckig (PC)                    grblHAL (Teensy)                │
│  ┌─────────────┐                ┌─────────────┐                 │
│  │  S-Curve    │ ─── G-code ──► │ Transparent │                 │
│  │  Planner    │                │  Pass-thru  │                 │
│  └─────────────┘                └─────────────┘                 │
│                                                                  │
│  Ruckig controls everything, grblHAL just generates steps!      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2. Bypass Configuration

#### 3.2.1. Junction Deviation ($11)
```
Default: $11 = 0.01 mm (corner slowdown)
Bypass:  $11 = 1.0 mm  (ignore corners)

Effect: grblHAL won't slow down at segment junctions
        because Ruckig already handles smooth transitions
```

#### 3.2.2. Acceleration Override ($120-$125)
```
Robot Physical Limit:  1000 mm/s² (or 10 rad/s² for joints)
Ruckig Max Accel:       800 mm/s² (safety margin)
grblHAL Max Accel:     5000 mm/s² (fictitious high value)

Effect: grblHAL accepts all acceleration commands from Ruckig
        without clamping or modification
```

#### 3.2.3. Motion Mode (G64)
```
G61 = Exact Stop Mode (BAD - stops at every point)
G64 = Continuous Mode (REQUIRED for streaming)

Always use G64 for trajectory streaming!
```

---

## 4. Task Breakdown

### 4.1. Task List

| ID | Task | Description | Priority | Dependencies |
|----|------|-------------|----------|--------------|
| **T-01** | Serial Interface | USB Serial communication class | P0 | None |
| **T-02** | Character Counting | Token bucket protocol | P0 | T-01 |
| **T-03** | G-code Generator | Convert joint positions to G-code | P0 | T-01 |
| **T-04** | Command Queue | Thread-safe command buffer | P0 | T-01 |
| **T-05** | Status Parser | Parse grblHAL status responses | P0 | T-01 |
| **T-06** | Position Feedback | Real-time position reporting | P0 | T-05 |
| **T-07** | Buffer Monitor | Track planner buffer level | P1 | T-05 |
| **T-08** | Firmware Config | grblHAL $ settings management | P0 | T-01 |
| **T-09** | Homing Sequence | Safe 6-axis homing order | P0 | T-08 |
| **T-10** | Emergency Stop | Hardware E-Stop handling | P0 | T-01 |
| **T-11** | Realtime Commands | Feed hold, Resume, Reset | P0 | T-01 |
| **T-12** | Arc Control | M3/M5 welding arc commands | P1 | T-03 |
| **T-13** | I/O Control | Coolant, Gas, Aux outputs | P1 | T-01 |
| **T-14** | Error Handling | Alarm recovery procedures | P0 | T-05 |
| **T-15** | Unit Tests | Communication tests | P0 | T-01 |

---

## 5. Implementation

### 5.1. Core Data Structures

```cpp
// grblhal_types.hpp

#pragma once
#include <array>
#include <string>
#include <cstdint>

namespace robot_controller {
namespace grblhal {

// Number of robot axes
constexpr size_t NUM_AXES = 6;

// Axis names for G-code (XYZABC)
constexpr std::array<char, NUM_AXES> AXIS_NAMES = {'X', 'Y', 'Z', 'A', 'B', 'C'};

/**
 * @brief grblHAL machine state
 */
enum class MachineState {
    IDLE,
    RUN,
    HOLD,
    JOG,
    HOMING,
    ALARM,
    CHECK,
    DOOR,
    TOOL,
    SLEEP,
    UNKNOWN
};

/**
 * @brief grblHAL alarm codes
 */
enum class AlarmCode {
    NONE = 0,
    HARD_LIMIT = 1,
    SOFT_LIMIT = 2,
    ABORT_CYCLE = 3,
    PROBE_FAIL_INITIAL = 4,
    PROBE_FAIL_CONTACT = 5,
    HOMING_FAIL_RESET = 6,
    HOMING_FAIL_DOOR = 7,
    HOMING_FAIL_PULLOFF = 8,
    HOMING_FAIL_APPROACH = 9,
    SPINDLE_CONTROL = 10,
    // ... more as needed
};

/**
 * @brief Status report from grblHAL
 */
struct StatusReport {
    MachineState state;
    std::array<double, NUM_AXES> machine_position;  // MPos
    std::array<double, NUM_AXES> work_position;     // WPos
    int planner_buffer_available;                    // Bf
    int rx_buffer_available;
    double feed_rate;
    double spindle_speed;
    uint32_t line_number;
    bool probe_triggered;
    uint32_t input_pins;  // Limit, probe, door states
};

/**
 * @brief G-code command with metadata
 */
struct GcodeCommand {
    std::string line;
    size_t byte_count;
    uint32_t sequence_id;
    bool is_realtime;  // ?, !, ~, etc.
};

/**
 * @brief grblHAL response types
 */
enum class ResponseType {
    OK,
    ERROR,
    ALARM,
    STATUS,
    FEEDBACK,  // [MSG:...], [PRB:...], etc.
    UNKNOWN
};

/**
 * @brief Parsed response from grblHAL
 */
struct Response {
    ResponseType type;
    int error_code;       // If ERROR
    int alarm_code;       // If ALARM
    StatusReport status;  // If STATUS
    std::string message;  // Raw or feedback message
};

/**
 * @brief grblHAL settings ($xxx values)
 */
struct GrblSettings {
    // Steps per unit
    std::array<double, NUM_AXES> steps_per_unit;     // $100-$105
    // Max rates
    std::array<double, NUM_AXES> max_rate;           // $110-$115
    // Max acceleration
    std::array<double, NUM_AXES> max_acceleration;   // $120-$125
    // Max travel
    std::array<double, NUM_AXES> max_travel;         // $130-$135
    // Junction deviation
    double junction_deviation;                        // $11
    // Arc tolerance
    double arc_tolerance;                             // $12
    // Report mask
    uint32_t status_report_mask;                      // $10
    // Homing
    bool homing_enabled;                              // $22
    uint32_t homing_dir_mask;                         // $23
    double homing_feed;                               // $24
    double homing_seek;                               // $25
    double homing_debounce;                           // $26
    double homing_pulloff;                            // $27
};

} // namespace grblhal
} // namespace robot_controller
```

### 5.2. Serial Interface

```cpp
// grblhal_serial.hpp

#pragma once
#include "grblhal_types.hpp"
#include <string>
#include <functional>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

namespace robot_controller {
namespace grblhal {

/**
 * @brief USB Serial interface to grblHAL on Teensy 4.1
 *
 * Implements Character Counting protocol for high-throughput
 * G-code streaming without buffer starvation.
 */
class GrblHALSerial {
public:
    /**
     * @brief Constructor
     * @param port Serial port name (e.g., "COM3" or "/dev/ttyACM0")
     * @param baud Baud rate (typically 2000000 for USB)
     */
    GrblHALSerial(const std::string& port, uint32_t baud = 2000000);

    /**
     * @brief Destructor
     */
    ~GrblHALSerial();

    /**
     * @brief Connect to grblHAL
     * @return true if connection successful
     */
    bool connect();

    /**
     * @brief Disconnect from grblHAL
     */
    void disconnect();

    /**
     * @brief Check if connected
     */
    bool isConnected() const { return connected_; }

    /**
     * @brief Send G-code command (queued)
     * @param gcode G-code line without newline
     * @return Sequence ID for tracking
     */
    uint32_t sendGcode(const std::string& gcode);

    /**
     * @brief Send real-time command (immediate, bypasses buffer)
     * @param cmd Real-time character (?, !, ~, 0x18, etc.)
     */
    void sendRealtime(char cmd);

    /**
     * @brief Request status report
     */
    void requestStatus();

    /**
     * @brief Soft reset grblHAL
     */
    void softReset();

    /**
     * @brief Feed hold (pause motion)
     */
    void feedHold();

    /**
     * @brief Resume from feed hold
     */
    void cycleStart();

    /**
     * @brief Get latest status report
     */
    StatusReport getStatus() const;

    /**
     * @brief Get available space in Teensy RX buffer
     */
    int getBufferAvailable() const { return buffer_available_; }

    /**
     * @brief Check if buffer has space for command
     */
    bool hasBufferSpace(size_t bytes) const;

    /**
     * @brief Callback types
     */
    using StatusCallback = std::function<void(const StatusReport&)>;
    using ErrorCallback = std::function<void(int error_code, const std::string& message)>;
    using AlarmCallback = std::function<void(AlarmCode code)>;

    /**
     * @brief Register callbacks
     */
    void onStatus(StatusCallback callback) { status_callback_ = callback; }
    void onError(ErrorCallback callback) { error_callback_ = callback; }
    void onAlarm(AlarmCallback callback) { alarm_callback_ = callback; }

    // Configuration
    static constexpr int RX_BUFFER_SIZE = 4096;  // Must match grblHAL config
    static constexpr int CHAR_COUNT_MARGIN = 50; // Safety margin

private:
    // Serial port handle (platform-specific)
    std::string port_name_;
    uint32_t baud_rate_;
    bool connected_;

#ifdef _WIN32
    void* serial_handle_;  // HANDLE
#else
    int serial_fd_;
#endif

    // Character counting state
    std::atomic<int> bytes_sent_;
    std::atomic<int> bytes_acknowledged_;
    std::atomic<int> buffer_available_;

    // Command queue
    std::queue<GcodeCommand> command_queue_;
    std::mutex queue_mutex_;
    uint32_t next_sequence_id_;

    // Thread management
    std::thread tx_thread_;
    std::thread rx_thread_;
    std::atomic<bool> running_;

    // Latest status
    StatusReport latest_status_;
    mutable std::mutex status_mutex_;

    // Callbacks
    StatusCallback status_callback_;
    ErrorCallback error_callback_;
    AlarmCallback alarm_callback_;

    // Internal methods
    void txThreadFunc();
    void rxThreadFunc();
    bool writeBytes(const void* data, size_t len);
    int readBytes(void* data, size_t maxLen);
    Response parseResponse(const std::string& line);
    StatusReport parseStatus(const std::string& line);
    MachineState parseState(const std::string& state_str);
    void processResponse(const Response& response);
    void updateBufferCount(size_t bytes_acked);
};

} // namespace grblhal
} // namespace robot_controller
```

### 5.3. GrblHALSerial Implementation

```cpp
// grblhal_serial.cpp

#include "grblhal_serial.hpp"
#include <spdlog/spdlog.h>
#include <sstream>
#include <regex>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

namespace robot_controller {
namespace grblhal {

GrblHALSerial::GrblHALSerial(const std::string& port, uint32_t baud)
    : port_name_(port)
    , baud_rate_(baud)
    , connected_(false)
    , bytes_sent_(0)
    , bytes_acknowledged_(0)
    , buffer_available_(RX_BUFFER_SIZE)
    , next_sequence_id_(1)
    , running_(false)
{
#ifdef _WIN32
    serial_handle_ = INVALID_HANDLE_VALUE;
#else
    serial_fd_ = -1;
#endif
}

GrblHALSerial::~GrblHALSerial() {
    disconnect();
}

bool GrblHALSerial::connect() {
#ifdef _WIN32
    serial_handle_ = CreateFileA(
        port_name_.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr
    );

    if (serial_handle_ == INVALID_HANDLE_VALUE) {
        spdlog::error("Failed to open serial port: {}", port_name_);
        return false;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(serial_handle_, &dcb);
    dcb.BaudRate = baud_rate_;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    SetCommState(serial_handle_, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(serial_handle_, &timeouts);
#else
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        spdlog::error("Failed to open serial port: {}", port_name_);
        return false;
    }

    struct termios tty;
    tcgetattr(serial_fd_, &tty);
    cfsetospeed(&tty, B2000000);
    cfsetispeed(&tty, B2000000);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;  // 0.5 second timeout
    tcsetattr(serial_fd_, TCSANOW, &tty);
#endif

    connected_ = true;
    running_ = true;

    // Start communication threads
    tx_thread_ = std::thread(&GrblHALSerial::txThreadFunc, this);
    rx_thread_ = std::thread(&GrblHALSerial::rxThreadFunc, this);

    // Wait for grblHAL startup message
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Request initial status
    requestStatus();

    spdlog::info("Connected to grblHAL on {}", port_name_);
    return true;
}

void GrblHALSerial::disconnect() {
    running_ = false;

    if (tx_thread_.joinable()) tx_thread_.join();
    if (rx_thread_.joinable()) rx_thread_.join();

#ifdef _WIN32
    if (serial_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_handle_);
        serial_handle_ = INVALID_HANDLE_VALUE;
    }
#else
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
#endif

    connected_ = false;
    spdlog::info("Disconnected from grblHAL");
}

uint32_t GrblHALSerial::sendGcode(const std::string& gcode) {
    GcodeCommand cmd;
    cmd.line = gcode + '\n';
    cmd.byte_count = cmd.line.size();
    cmd.sequence_id = next_sequence_id_++;
    cmd.is_realtime = false;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(cmd);
    }

    return cmd.sequence_id;
}

void GrblHALSerial::sendRealtime(char cmd) {
    // Real-time commands bypass buffer (no newline needed)
    writeBytes(&cmd, 1);
}

void GrblHALSerial::requestStatus() {
    sendRealtime('?');
}

void GrblHALSerial::softReset() {
    sendRealtime(0x18);  // Ctrl+X
    bytes_sent_ = 0;
    bytes_acknowledged_ = 0;
    buffer_available_ = RX_BUFFER_SIZE;
}

void GrblHALSerial::feedHold() {
    sendRealtime('!');
}

void GrblHALSerial::cycleStart() {
    sendRealtime('~');
}

StatusReport GrblHALSerial::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_status_;
}

bool GrblHALSerial::hasBufferSpace(size_t bytes) const {
    return buffer_available_ >= static_cast<int>(bytes + CHAR_COUNT_MARGIN);
}

void GrblHALSerial::txThreadFunc() {
    while (running_) {
        GcodeCommand cmd;
        bool have_cmd = false;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!command_queue_.empty()) {
                cmd = command_queue_.front();

                // Character counting: check buffer space
                if (hasBufferSpace(cmd.byte_count)) {
                    command_queue_.pop();
                    have_cmd = true;
                }
            }
        }

        if (have_cmd) {
            if (writeBytes(cmd.line.c_str(), cmd.byte_count)) {
                bytes_sent_ += cmd.byte_count;
                buffer_available_ -= cmd.byte_count;
            }
        } else {
            // No command or buffer full, wait a bit
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

void GrblHALSerial::rxThreadFunc() {
    std::string line_buffer;
    char read_buffer[256];

    while (running_) {
        int bytes_read = readBytes(read_buffer, sizeof(read_buffer) - 1);

        if (bytes_read > 0) {
            read_buffer[bytes_read] = '\0';

            for (int i = 0; i < bytes_read; i++) {
                char c = read_buffer[i];

                if (c == '\n' || c == '\r') {
                    if (!line_buffer.empty()) {
                        Response response = parseResponse(line_buffer);
                        processResponse(response);
                        line_buffer.clear();
                    }
                } else {
                    line_buffer += c;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

Response GrblHALSerial::parseResponse(const std::string& line) {
    Response response;
    response.message = line;

    if (line == "ok") {
        response.type = ResponseType::OK;
        // Acknowledge one command (estimate ~20 bytes average)
        // In real implementation, track actual byte counts
    }
    else if (line.substr(0, 6) == "error:") {
        response.type = ResponseType::ERROR;
        response.error_code = std::stoi(line.substr(6));
    }
    else if (line.substr(0, 6) == "ALARM:") {
        response.type = ResponseType::ALARM;
        response.alarm_code = std::stoi(line.substr(6));
    }
    else if (line[0] == '<' && line.back() == '>') {
        response.type = ResponseType::STATUS;
        response.status = parseStatus(line);
    }
    else if (line[0] == '[') {
        response.type = ResponseType::FEEDBACK;
    }
    else {
        response.type = ResponseType::UNKNOWN;
    }

    return response;
}

StatusReport GrblHALSerial::parseStatus(const std::string& line) {
    // Parse: <Idle|MPos:0.000,0.000,0.000,0.000,0.000,0.000|Bf:15,128|...>
    StatusReport status = {};

    // Remove < and >
    std::string content = line.substr(1, line.size() - 2);

    // Split by |
    std::istringstream ss(content);
    std::string field;
    bool first = true;

    while (std::getline(ss, field, '|')) {
        if (first) {
            status.state = parseState(field);
            first = false;
        }
        else if (field.substr(0, 5) == "MPos:") {
            // Parse machine position
            std::string pos_str = field.substr(5);
            std::istringstream pos_ss(pos_str);
            std::string val;
            int axis = 0;
            while (std::getline(pos_ss, val, ',') && axis < NUM_AXES) {
                status.machine_position[axis++] = std::stod(val);
            }
        }
        else if (field.substr(0, 5) == "WPos:") {
            // Parse work position
            std::string pos_str = field.substr(5);
            std::istringstream pos_ss(pos_str);
            std::string val;
            int axis = 0;
            while (std::getline(pos_ss, val, ',') && axis < NUM_AXES) {
                status.work_position[axis++] = std::stod(val);
            }
        }
        else if (field.substr(0, 3) == "Bf:") {
            // Parse buffer state: Bf:planner,rx
            std::string bf_str = field.substr(3);
            size_t comma = bf_str.find(',');
            status.planner_buffer_available = std::stoi(bf_str.substr(0, comma));
            status.rx_buffer_available = std::stoi(bf_str.substr(comma + 1));
        }
        else if (field.substr(0, 2) == "F:") {
            status.feed_rate = std::stod(field.substr(2));
        }
        else if (field.substr(0, 2) == "S:") {
            status.spindle_speed = std::stod(field.substr(2));
        }
    }

    return status;
}

MachineState GrblHALSerial::parseState(const std::string& state_str) {
    // Handle substates like "Hold:0"
    std::string state = state_str;
    size_t colon = state.find(':');
    if (colon != std::string::npos) {
        state = state.substr(0, colon);
    }

    if (state == "Idle") return MachineState::IDLE;
    if (state == "Run") return MachineState::RUN;
    if (state == "Hold") return MachineState::HOLD;
    if (state == "Jog") return MachineState::JOG;
    if (state == "Home") return MachineState::HOMING;
    if (state == "Alarm") return MachineState::ALARM;
    if (state == "Check") return MachineState::CHECK;
    if (state == "Door") return MachineState::DOOR;
    if (state == "Tool") return MachineState::TOOL;
    if (state == "Sleep") return MachineState::SLEEP;

    return MachineState::UNKNOWN;
}

void GrblHALSerial::processResponse(const Response& response) {
    switch (response.type) {
        case ResponseType::OK:
            // Acknowledge bytes (simplified - real impl tracks per-command)
            updateBufferCount(20);  // Average command size estimate
            break;

        case ResponseType::ERROR:
            spdlog::error("grblHAL error: {} - {}", response.error_code, response.message);
            if (error_callback_) {
                error_callback_(response.error_code, response.message);
            }
            updateBufferCount(20);  // Error still consumes a command
            break;

        case ResponseType::ALARM:
            spdlog::error("grblHAL ALARM: {}", response.alarm_code);
            if (alarm_callback_) {
                alarm_callback_(static_cast<AlarmCode>(response.alarm_code));
            }
            break;

        case ResponseType::STATUS:
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                latest_status_ = response.status;
                // Update buffer tracking from actual status
                buffer_available_ = response.status.rx_buffer_available;
            }
            if (status_callback_) {
                status_callback_(response.status);
            }
            break;

        default:
            break;
    }
}

void GrblHALSerial::updateBufferCount(size_t bytes_acked) {
    bytes_acknowledged_ += bytes_acked;
    buffer_available_ += bytes_acked;
    if (buffer_available_ > RX_BUFFER_SIZE) {
        buffer_available_ = RX_BUFFER_SIZE;
    }
}

bool GrblHALSerial::writeBytes(const void* data, size_t len) {
#ifdef _WIN32
    DWORD written;
    return WriteFile(serial_handle_, data, static_cast<DWORD>(len), &written, nullptr) && written == len;
#else
    return write(serial_fd_, data, len) == static_cast<ssize_t>(len);
#endif
}

int GrblHALSerial::readBytes(void* data, size_t maxLen) {
#ifdef _WIN32
    DWORD read;
    if (ReadFile(serial_handle_, data, static_cast<DWORD>(maxLen), &read, nullptr)) {
        return static_cast<int>(read);
    }
    return 0;
#else
    return static_cast<int>(read(serial_fd_, data, maxLen));
#endif
}

} // namespace grblhal
} // namespace robot_controller
```

### 5.4. G-code Generator

```cpp
// gcode_generator.hpp

#pragma once
#include "grblhal_types.hpp"
#include <array>
#include <string>
#include <sstream>
#include <iomanip>

namespace robot_controller {
namespace grblhal {

/**
 * @brief Converts joint positions to G-code commands
 *
 * Generates compact G-code for high-frequency streaming from Ruckig OTG.
 */
class GcodeGenerator {
public:
    /**
     * @brief Constructor
     * @param precision Decimal places for coordinates (3-4 recommended)
     */
    explicit GcodeGenerator(int precision = 4);

    /**
     * @brief Generate G1 move command
     * @param positions Joint positions in degrees or radians
     * @param feed_rate Feed rate in deg/min or rad/min
     * @return G-code string without newline
     */
    std::string generateMove(const std::array<double, NUM_AXES>& positions,
                             double feed_rate);

    /**
     * @brief Generate compact incremental move (G91 mode)
     * @param deltas Position deltas
     * @param feed_rate Feed rate
     * @return G-code string
     */
    std::string generateIncrementalMove(const std::array<double, NUM_AXES>& deltas,
                                         double feed_rate);

    /**
     * @brief Generate dwell command
     * @param seconds Dwell time
     * @return G-code string
     */
    std::string generateDwell(double seconds);

    /**
     * @brief Generate arc on command (M3)
     * @param power Arc power (0-100 or actual current)
     * @return G-code string
     */
    std::string generateArcOn(double power = 100);

    /**
     * @brief Generate arc off command (M5)
     * @return G-code string
     */
    std::string generateArcOff();

    /**
     * @brief Generate gas on command (M7 - mist/gas flow)
     * @return G-code string
     */
    std::string generateGasOn();

    /**
     * @brief Generate gas off command (M9)
     * @return G-code string
     */
    std::string generateGasOff();

    /**
     * @brief Set to absolute mode (G90)
     */
    std::string setAbsoluteMode();

    /**
     * @brief Set to incremental mode (G91)
     */
    std::string setIncrementalMode();

    /**
     * @brief Set continuous path mode (G64)
     */
    std::string setContinuousMode();

    /**
     * @brief Configure unit conversion
     * @param scale Scale factor (e.g., 180/PI for rad to deg)
     */
    void setUnitScale(double scale) { unit_scale_ = scale; }

    /**
     * @brief Track last sent positions for modal optimization
     */
    void updateLastPosition(const std::array<double, NUM_AXES>& positions);

private:
    int precision_;
    double unit_scale_;
    std::array<double, NUM_AXES> last_positions_;
    bool positions_known_;
    double last_feed_rate_;

    // Optimization: skip unchanged axes
    bool shouldIncludeAxis(int axis, double value) const;

    // Format number with minimal characters
    std::string formatNumber(double value) const;
};

} // namespace grblhal
} // namespace robot_controller
```

### 5.5. GcodeGenerator Implementation

```cpp
// gcode_generator.cpp

#include "gcode_generator.hpp"
#include <cmath>

namespace robot_controller {
namespace grblhal {

GcodeGenerator::GcodeGenerator(int precision)
    : precision_(precision)
    , unit_scale_(1.0)
    , positions_known_(false)
    , last_feed_rate_(0.0)
{
    last_positions_.fill(0.0);
}

std::string GcodeGenerator::generateMove(const std::array<double, NUM_AXES>& positions,
                                          double feed_rate) {
    std::ostringstream ss;

    // Only include G1 if modal state might be different
    // In streaming mode, we rely on G64 being set once

    bool first = true;
    for (size_t i = 0; i < NUM_AXES; i++) {
        double value = positions[i] * unit_scale_;

        // Optimization: skip if value unchanged (within tolerance)
        if (positions_known_ && std::abs(value - last_positions_[i]) < 0.0001) {
            continue;
        }

        if (!first) {
            // No space needed between axes for compactness
        }
        first = false;

        ss << AXIS_NAMES[i] << formatNumber(value);
    }

    // Only include F if feed rate changed
    if (std::abs(feed_rate - last_feed_rate_) > 0.01) {
        ss << 'F' << formatNumber(feed_rate);
        last_feed_rate_ = feed_rate;
    }

    // Update tracked positions
    for (size_t i = 0; i < NUM_AXES; i++) {
        last_positions_[i] = positions[i] * unit_scale_;
    }
    positions_known_ = true;

    return ss.str();
}

std::string GcodeGenerator::generateIncrementalMove(
    const std::array<double, NUM_AXES>& deltas, double feed_rate) {

    std::ostringstream ss;

    for (size_t i = 0; i < NUM_AXES; i++) {
        double value = deltas[i] * unit_scale_;

        // Skip zero deltas
        if (std::abs(value) < 0.0001) {
            continue;
        }

        ss << AXIS_NAMES[i] << formatNumber(value);
    }

    if (std::abs(feed_rate - last_feed_rate_) > 0.01) {
        ss << 'F' << formatNumber(feed_rate);
        last_feed_rate_ = feed_rate;
    }

    return ss.str();
}

std::string GcodeGenerator::generateDwell(double seconds) {
    std::ostringstream ss;
    ss << "G4P" << std::fixed << std::setprecision(3) << seconds;
    return ss.str();
}

std::string GcodeGenerator::generateArcOn(double power) {
    std::ostringstream ss;
    ss << "M3S" << static_cast<int>(power);
    return ss.str();
}

std::string GcodeGenerator::generateArcOff() {
    return "M5";
}

std::string GcodeGenerator::generateGasOn() {
    return "M7";
}

std::string GcodeGenerator::generateGasOff() {
    return "M9";
}

std::string GcodeGenerator::setAbsoluteMode() {
    return "G90";
}

std::string GcodeGenerator::setIncrementalMode() {
    return "G91";
}

std::string GcodeGenerator::setContinuousMode() {
    return "G64";
}

void GcodeGenerator::updateLastPosition(const std::array<double, NUM_AXES>& positions) {
    for (size_t i = 0; i < NUM_AXES; i++) {
        last_positions_[i] = positions[i] * unit_scale_;
    }
    positions_known_ = true;
}

std::string GcodeGenerator::formatNumber(double value) const {
    std::ostringstream ss;

    // Remove trailing zeros for compactness
    ss << std::fixed << std::setprecision(precision_) << value;
    std::string str = ss.str();

    // Remove trailing zeros
    size_t dot = str.find('.');
    if (dot != std::string::npos) {
        size_t last = str.find_last_not_of('0');
        if (last != std::string::npos && last > dot) {
            str = str.substr(0, last + 1);
        } else if (last == dot) {
            str = str.substr(0, dot);  // Remove decimal point too
        }
    }

    return str;
}

} // namespace grblhal
} // namespace robot_controller
```

---

## 6. Firmware Configuration

### 6.1. my_machine.h Configuration

```c
// my_machine.h - grblHAL configuration for 6-DOF Robot

#ifndef MY_MACHINE_H
#define MY_MACHINE_H

//=============================================================================
// AXIS CONFIGURATION
//=============================================================================

// Enable 6 axes (XYZABC = Joints 1-6)
#define N_AXIS 6

// Axis naming for display
#define AXIS_NAMES_XYZABC

//=============================================================================
// BUFFER SIZES (Critical for streaming!)
//=============================================================================

// RX buffer - must be large for character counting
#define RX_BUFFER_SIZE 4096

// Planner buffer - stores motion segments
#define BLOCK_BUFFER_SIZE 256

// Line buffer for G-code parsing
#define LINE_BUFFER_SIZE 256

//=============================================================================
// COMMUNICATION
//=============================================================================

// USB CDC communication
#define USB_SERIAL_CDC 1

// Baud rate (USB is actually fixed at 12Mbps, this is for compatibility)
#define BAUD_RATE 2000000

//=============================================================================
// MOTION SETTINGS
//=============================================================================

// Disable step pulse delay for maximum speed
#define STEP_PULSE_DELAY 0

// Step pulse duration in microseconds
#define STEP_PULSE_MICROSECONDS 5

// Enable soft limits
#define DEFAULT_SOFT_LIMITS_ENABLE 1

// Disable hard limits by default (use software limits for robots)
#define DEFAULT_HARD_LIMITS_ENABLE 0

//=============================================================================
// HOMING
//=============================================================================

// Enable homing
#define DEFAULT_HOMING_ENABLE 1

// Homing direction mask (depends on robot configuration)
// Bit 0 = X, Bit 1 = Y, etc. 1 = negative direction
#define DEFAULT_HOMING_DIR_MASK 0b00111111  // All negative

// Homing order (customize for robot safety!)
// See HOMING_CYCLE_x definitions

//=============================================================================
// SAFETY
//=============================================================================

// Enable E-Stop input
#define ESTOP_ENABLE 1

// Safety door support (optional)
#define SAFETY_DOOR_ENABLE 0

//=============================================================================
// SPINDLE (Used for Arc Control)
//=============================================================================

// PWM spindle for analog 0-10V output
#define SPINDLE_PWM_TIMER_N 1

// Spindle enable pin
#define SPINDLE_ENABLE_PIN 12

// Spindle PWM pin
#define SPINDLE_PWM_PIN 13

//=============================================================================
// PROBE (Used for Touch Sensing)
//=============================================================================

// Enable probe input
#define PROBE_ENABLE 1

// Probe pin
#define PROBE_PIN 23

#endif // MY_MACHINE_H
```

### 6.2. grblHAL $ Settings

```yaml
# grblHAL Settings for 6-DOF Robot with Ruckig Bypass
# Apply via serial console: $xxx=value

# Steps per degree for each joint
$100: 88.889   # J1 steps/deg (32000 steps/360°)
$101: 88.889   # J2 steps/deg
$102: 88.889   # J3 steps/deg
$103: 88.889   # J4 steps/deg
$104: 88.889   # J5 steps/deg
$105: 88.889   # J6 steps/deg

# Max velocity (deg/min) - HIGH for bypass
$110: 36000    # J1 max rate (600 deg/s = 36000 deg/min)
$111: 36000    # J2 max rate
$112: 36000    # J3 max rate
$113: 72000    # J4 max rate (wrist faster)
$114: 72000    # J5 max rate
$115: 72000    # J6 max rate

# Max acceleration (deg/s²) - FICTITIOUS HIGH for bypass
$120: 5000     # J1 max accel (much higher than Ruckig uses)
$121: 5000     # J2 max accel
$122: 5000     # J3 max accel
$123: 10000    # J4 max accel
$124: 10000    # J5 max accel
$125: 10000    # J6 max accel

# Max travel (degrees) - joint limits
$130: 360      # J1 range
$131: 180      # J2 range
$132: 180      # J3 range
$133: 360      # J4 range
$134: 180      # J5 range
$135: 360      # J6 range

# Junction deviation - HIGH for bypass (critical!)
$11: 1.0       # 1.0 deg deviation (essentially disable corner slowdown)

# Arc tolerance
$12: 0.002     # Not used for robot joints

# Status report mask
$10: 511       # Report all: MPos, Bf, FS, Pn, etc.

# Soft limits
$20: 1         # Enable soft limits

# Hard limits
$21: 0         # Disable hard limits (use soft limits for robots)

# Homing
$22: 1         # Enable homing
$23: 63        # Homing direction mask (all negative)
$24: 25        # Homing feed rate (deg/min)
$25: 500       # Homing seek rate (deg/min)
$26: 250       # Homing debounce (ms)
$27: 1.0       # Homing pull-off (deg)

# Laser mode (use for continuous arc control)
$32: 0         # Disabled for normal arc control
```

---

## 7. Configuration

### 7.1. YAML Configuration File

```yaml
# config/grblhal_config.yaml

grblhal:
  # Serial port settings
  serial:
    port: "COM3"              # Windows example
    # port: "/dev/ttyACM0"    # Linux example
    baud_rate: 2000000        # USB effectively ignores this

  # Buffer management
  buffers:
    rx_buffer_size: 4096      # Must match firmware
    character_count_margin: 50 # Safety margin for flow control
    status_poll_interval_ms: 50 # How often to poll status

  # Unit conversion
  units:
    joint_unit: "degrees"     # or "radians"
    # Ruckig uses radians, grblHAL expects degrees
    radians_to_degrees: 57.2957795  # 180/PI

  # Streaming settings
  streaming:
    stream_interval_ms: 10    # 100Hz streaming rate
    min_buffer_threshold: 100 # Min buffer before pause streaming
    max_queue_size: 1000      # Max commands in PC queue

  # Safety
  safety:
    estop_pin: 19
    enable_soft_limits: true
    enable_hard_limits: false

  # Homing sequence (safe order for robot)
  homing:
    enabled: true
    # Order: wrist first (least mass), then arm, then base
    sequence:
      - axes: [4, 5]    # J5, J6 (wrist roll/pitch)
        direction: "negative"
      - axes: [3]       # J4 (wrist rotate)
        direction: "negative"
      - axes: [1, 2]    # J2, J3 (arm)
        direction: "negative"
      - axes: [0]       # J1 (base)
        direction: "negative"

  # Default settings (applied on connection)
  default_settings:
    junction_deviation: 1.0   # $11 for bypass
    motion_mode: "G64"        # Continuous path
    coordinate_mode: "G90"    # Absolute

  # Welding I/O mapping
  welding:
    arc_on_command: "M3"
    arc_off_command: "M5"
    gas_on_command: "M7"
    gas_off_command: "M9"
    arc_power_scale: 1.0      # S value = actual current

  # Logging
  logging:
    log_commands: false       # Enable for debugging (high volume!)
    log_responses: true
    log_status: false
```

---

## 8. Testing

### 8.1. Unit Test Cases

```cpp
// test_grblhal.cpp

#include <gtest/gtest.h>
#include "grblhal/grblhal_serial.hpp"
#include "grblhal/gcode_generator.hpp"

using namespace robot_controller::grblhal;

class GcodeGeneratorTest : public ::testing::Test {
protected:
    GcodeGenerator gen{4};
};

TEST_F(GcodeGeneratorTest, GenerateMove6Axis) {
    std::array<double, 6> positions = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    std::string gcode = gen.generateMove(positions, 1000.0);

    EXPECT_NE(gcode.find("X10"), std::string::npos);
    EXPECT_NE(gcode.find("Y20"), std::string::npos);
    EXPECT_NE(gcode.find("Z30"), std::string::npos);
    EXPECT_NE(gcode.find("A40"), std::string::npos);
    EXPECT_NE(gcode.find("B50"), std::string::npos);
    EXPECT_NE(gcode.find("C60"), std::string::npos);
    EXPECT_NE(gcode.find("F1000"), std::string::npos);
}

TEST_F(GcodeGeneratorTest, SkipUnchangedAxes) {
    std::array<double, 6> pos1 = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    gen.generateMove(pos1, 1000.0);

    // Only change axis 0 and 3
    std::array<double, 6> pos2 = {11.0, 20.0, 30.0, 41.0, 50.0, 60.0};
    std::string gcode = gen.generateMove(pos2, 1000.0);

    EXPECT_NE(gcode.find("X11"), std::string::npos);
    EXPECT_NE(gcode.find("A41"), std::string::npos);
    EXPECT_EQ(gcode.find("Y"), std::string::npos);  // Y unchanged
    EXPECT_EQ(gcode.find("Z"), std::string::npos);  // Z unchanged
    EXPECT_EQ(gcode.find("F"), std::string::npos);  // F unchanged
}

TEST_F(GcodeGeneratorTest, CompactNumberFormat) {
    std::array<double, 6> positions = {10.5000, 20.0, 30.123, 40.0, 50.0, 60.0};
    std::string gcode = gen.generateMove(positions, 1000.0);

    // Should not have trailing zeros
    EXPECT_NE(gcode.find("X10.5"), std::string::npos);  // Not X10.5000
    EXPECT_NE(gcode.find("Y20"), std::string::npos);    // Not Y20.0000
}

TEST_F(GcodeGeneratorTest, ArcCommands) {
    EXPECT_EQ(gen.generateArcOn(100), "M3S100");
    EXPECT_EQ(gen.generateArcOff(), "M5");
    EXPECT_EQ(gen.generateGasOn(), "M7");
    EXPECT_EQ(gen.generateGasOff(), "M9");
}

class StatusParserTest : public ::testing::Test {
protected:
    // Would need mock serial or parse function extracted
};

TEST_F(StatusParserTest, ParseIdleStatus) {
    std::string status_line = "<Idle|MPos:0.000,0.000,0.000,0.000,0.000,0.000|Bf:255,4096|F:0|S:0>";
    // Would test parseStatus function
}

// Integration test (requires hardware or mock)
class GrblHALSerialTest : public ::testing::Test {
protected:
    // Mock serial port for testing
};

TEST_F(GrblHALSerialTest, CharacterCountingProtocol) {
    // Test that commands are queued and sent respecting buffer limits
    // Would use mock serial port
}
```

---

## 9. References

### 9.1. Research Documents
| Document | Path |
|----------|------|
| Tối ưu grblHAL cho Robot 6-DOF | ressearch_doc_md/ |

### 9.2. External Resources
| Resource | URL |
|----------|-----|
| grblHAL GitHub | https://github.com/grblHAL |
| grblHAL iMXRT1062 Driver | https://github.com/grblHAL/iMXRT1062 |
| Teensy 4.1 | https://www.pjrc.com/store/teensy41.html |
| grblHAL Wiki | https://github.com/grblHAL/core/wiki |

---

## APPENDIX

### A. grblHAL Real-time Commands

| Character | Name | Function |
|-----------|------|----------|
| `?` | Status Query | Request current status report |
| `!` | Feed Hold | Pause motion (decelerate to stop) |
| `~` | Cycle Start | Resume from hold |
| `0x18` | Soft Reset | Reset grblHAL (Ctrl+X) |
| `0x84` | Safety Door | Toggle safety door state |
| `0x85` | Jog Cancel | Cancel current jog |
| `0x90-0x9F` | Feed Override | Real-time feed rate adjustment |
| `0xA0-0xA3` | Spindle Override | Real-time spindle speed |

### B. Error Codes

| Code | Description |
|------|-------------|
| 1 | G-code command letter expected |
| 2 | Bad number format |
| 3 | Invalid $ statement |
| ... | See grblHAL documentation |
| 20 | Soft limit triggered |
| 21 | Max step rate exceeded |

### C. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
