/**
 * @file FirmwareInterface.hpp
 * @brief High-level interface to robot firmware (grblHAL)
 */

#pragma once

#include "FirmwareProtocol.hpp"
#include "SerialPort.hpp"
#include <functional>
#include <future>
#include <map>

namespace robot_controller {
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
    bool autoConnect();

    // ========================================================================
    // Status
    // ========================================================================

    MachineStatus getStatus();
    void startStatusPolling(int intervalMs = 50);
    void stopStatusPolling();
    const MachineStatus& getCachedStatus() const { return cachedStatus_; }

    // ========================================================================
    // Motion Commands
    // ========================================================================

    bool moveJoints(
        const std::array<double, NUM_AXES>& positions,
        double feedRate,
        bool blocking = false);

    bool moveLinear(
        const std::array<double, 3>& position,
        double feedRate,
        bool blocking = false);

    bool moveRapid(const std::array<double, NUM_AXES>& positions);
    bool jogStart(int axis, int direction, double speed);
    bool jogStop();
    bool stopMotion();
    bool feedHold();
    bool cycleStart();

    // ========================================================================
    // Homing
    // ========================================================================

    bool homeAll(bool blocking = true);
    bool homeAxis(int axis, bool blocking = true);
    bool setHome();
    bool isHomed(int axis) const;
    bool isAllHomed() const;

    // ========================================================================
    // Position
    // ========================================================================

    std::array<double, NUM_AXES> getJointPositions();
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

    std::string sendGCode(const std::string& command, int timeoutMs = 1000);
    bool sendGCodeOk(const std::string& command, int timeoutMs = 1000);

    // ========================================================================
    // Feed Override
    // ========================================================================

    void setFeedOverride(int percent);
    void setRapidOverride(int percent);

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
    mutable std::mutex statusMutex_;

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
        // Log error - could add callback here
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
    char query = GCode::STATUS_QUERY;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&query), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
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
    char query = GCode::STATUS_QUERY;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&query), 1);

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
            char query = GCode::STATUS_QUERY;
            serial_.writeBytes(reinterpret_cast<const uint8_t*>(&query), 1);
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

inline bool FirmwareInterface::moveLinear(
    const std::array<double, 3>& position,
    double feedRate,
    bool blocking) {

    if (!isConnected()) return false;

    // Build command with only XYZ
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "G1 X%.4f Y%.4f Z%.4f F%.1f",
             position[0], position[1], position[2], feedRate * 60.0);

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

    const char axisLetters[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};

    char cmd[64];
    double distance = direction * 1000.0;
    snprintf(cmd, sizeof(cmd), "$J=%c%.3fF%.1f",
             axisLetters[axis], distance, speed * 60.0);

    serial_.writeLine(cmd);
    return true;
}

inline bool FirmwareInterface::jogStop() {
    if (!isConnected()) return false;

    char cancel = GCode::JOG_CANCEL;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&cancel), 1);
    return true;
}

inline bool FirmwareInterface::stopMotion() {
    if (!isConnected()) return false;

    // Send feed hold followed by soft reset
    char hold = GCode::FEED_HOLD;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&hold), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    char reset = GCode::SOFT_RESET;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&reset), 1);

    return true;
}

inline bool FirmwareInterface::feedHold() {
    if (!isConnected()) return false;
    char hold = GCode::FEED_HOLD;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&hold), 1);
    return true;
}

inline bool FirmwareInterface::cycleStart() {
    if (!isConnected()) return false;
    char start = GCode::CYCLE_START;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&start), 1);
    return true;
}

inline bool FirmwareInterface::homeAll(bool blocking) {
    if (!isConnected()) return false;

    serial_.writeLine("$H");

    if (blocking) {
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
    std::lock_guard<std::mutex> lock(statusMutex_);
    return cachedStatus_.isHomed(axis);
}

inline bool FirmwareInterface::isAllHomed() const {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return cachedStatus_.isAllHomed();
}

inline std::array<double, NUM_AXES> FirmwareInterface::getJointPositions() {
    std::array<double, NUM_AXES> positions;

    std::lock_guard<std::mutex> lock(statusMutex_);
    for (int i = 0; i < NUM_AXES; ++i) {
        positions[i] = cachedStatus_.position.axis[i] / 1000.0;
    }

    return positions;
}

inline bool FirmwareInterface::setPosition(const std::array<double, NUM_AXES>& positions) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "G92 X%.4f Y%.4f Z%.4f A%.4f B%.4f C%.4f",
             positions[0], positions[1], positions[2],
             positions[3], positions[4], positions[5]);

    return sendGCodeOk(cmd);
}

inline uint16_t FirmwareInterface::getDigitalInputs() {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return cachedStatus_.io.digitalInputs;
}

inline bool FirmwareInterface::setDigitalOutputs(uint16_t outputs) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "M64 P%d", outputs);
    return sendGCodeOk(cmd);
}

inline uint16_t FirmwareInterface::getAnalogInput(int channel) {
    if (channel < 0 || channel >= 4) return 0;

    std::lock_guard<std::mutex> lock(statusMutex_);
    return cachedStatus_.io.analogInputs[channel];
}

inline LimitStatus FirmwareInterface::getLimitStatus() {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return cachedStatus_.limits;
}

inline bool FirmwareInterface::clearAlarm() {
    if (!isConnected()) return false;

    serial_.writeLine("$X");
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

    char cmd = GCode::FEED_100;
    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&cmd), 1);

    while (percent > 100) {
        cmd = GCode::FEED_INCREASE_10;
        serial_.writeBytes(reinterpret_cast<const uint8_t*>(&cmd), 1);
        percent -= 10;
    }
    while (percent < 100) {
        cmd = GCode::FEED_DECREASE_10;
        serial_.writeBytes(reinterpret_cast<const uint8_t*>(&cmd), 1);
        percent += 10;
    }
}

inline void FirmwareInterface::setRapidOverride(int percent) {
    if (!isConnected()) return;

    char cmd;
    if (percent <= 25) {
        cmd = GCode::RAPID_25;
    } else if (percent <= 50) {
        cmd = GCode::RAPID_50;
    } else {
        cmd = GCode::RAPID_100;
    }

    serial_.writeBytes(reinterpret_cast<const uint8_t*>(&cmd), 1);
}

inline void FirmwareInterface::handleLine(const std::string& line) {
    if (line.empty()) return;

    // Status response
    if (line[0] == '<') {
        MachineStatus status;
        if (parseStatusResponse(line, status)) {
            updateStatus(status);
        }

        {
            std::lock_guard<std::mutex> lock(responseMutex_);
            responseReceived_ = true;
        }
        responseCv_.notify_one();
        return;
    }

    // Response message
    {
        std::lock_guard<std::mutex> lock(responseMutex_);
        lastResponse_ = line;
        responseReceived_ = true;
    }
    responseCv_.notify_one();

    // Check for alarm
    if (line.find("ALARM") != std::string::npos) {
        if (alarmCallback_) {
            std::lock_guard<std::mutex> lock(statusMutex_);
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

    if (statusCallback_) {
        statusCallback_(status);
    }

    if (status.alarm != prevAlarm && status.alarm != AlarmCode::NONE) {
        if (alarmCallback_) {
            alarmCallback_(status.alarm);
        }
    }
}

} // namespace firmware
} // namespace robot_controller
