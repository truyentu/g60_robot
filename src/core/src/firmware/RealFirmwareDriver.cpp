#include "RealFirmwareDriver.hpp"
#include "../logging/Logger.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace robot_controller {
namespace firmware {

RealFirmwareDriver::RealFirmwareDriver() {
    // Set up status callback to cache positions
    m_firmware.setStatusCallback([this](const MachineStatus& status) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        for (int i = 0; i < DRIVER_NUM_AXES; i++) {
            m_positions[i] = status.position.axis[i] / 1000.0;  // int32 * 1000 -> degrees
        }

        switch (status.state) {
            case MachineState::IDLE:   m_lastState = "Idle"; break;
            case MachineState::RUN:    m_lastState = "Run"; break;
            case MachineState::HOLD:   m_lastState = "Hold"; break;
            case MachineState::JOG:    m_lastState = "Jog"; break;
            case MachineState::HOMING: m_lastState = "Homing"; break;
            case MachineState::ALARM:  m_lastState = "Alarm"; break;
            default:                   m_lastState = "Unknown"; break;
        }
    });

    LOG_INFO("RealFirmwareDriver created");
}

RealFirmwareDriver::~RealFirmwareDriver() {
    disconnect();
}

bool RealFirmwareDriver::connect(const std::string& portName, int baudRate) {
    LOG_INFO("RealFirmwareDriver: Connecting to {} at {} baud...", portName, baudRate);

    if (!m_firmware.connect(portName, baudRate)) {
        LOG_ERROR("RealFirmwareDriver: Failed to connect to {}", portName);
        return false;
    }

    m_portName = portName;

    // Start status polling at 50ms interval
    m_firmware.startStatusPolling(50);

    LOG_INFO("RealFirmwareDriver: Connected to {}", portName);
    return true;
}

void RealFirmwareDriver::disconnect() {
    if (m_firmware.isConnected()) {
        LOG_INFO("RealFirmwareDriver: Disconnecting from {}", m_portName);
        m_firmware.stopStatusPolling();
        m_firmware.disconnect();
        m_portName.clear();
    }
}

bool RealFirmwareDriver::autoConnect() {
    LOG_INFO("RealFirmwareDriver: Auto-connecting...");

    if (m_firmware.autoConnect()) {
        m_portName = "auto";  // We don't know exact port from autoConnect
        m_firmware.startStatusPolling(50);
        LOG_INFO("RealFirmwareDriver: Auto-connected successfully");
        return true;
    }

    LOG_WARN("RealFirmwareDriver: Auto-connect failed, no Teensy found");
    return false;
}

bool RealFirmwareDriver::isConnected() const {
    return m_firmware.isConnected();
}

bool RealFirmwareDriver::sendCommand(const std::string& gcode) {
    if (!m_firmware.isConnected()) {
        LOG_WARN("RealFirmwareDriver: Not connected, cannot send command");
        return false;
    }

    // Handle real-time single-char commands
    if (gcode.size() == 1) {
        char c = gcode[0];
        if (c == '?' || c == '\x85' || c == '!' || c == '~' || c == '\x18') {
            // These are real-time commands, send as raw bytes
            std::string response = m_firmware.sendGCode(gcode, 500);
            if (!response.empty()) {
                std::lock_guard<std::mutex> lock(m_responseMutex);
                m_responses.push(response);
            }
            return true;
        }
    }

    // Send G-code and capture response
    std::string response = m_firmware.sendGCode(gcode);
    if (!response.empty()) {
        std::lock_guard<std::mutex> lock(m_responseMutex);
        m_responses.push(response);
    }

    return true;
}

std::string RealFirmwareDriver::getResponse() {
    std::lock_guard<std::mutex> lock(m_responseMutex);
    if (!m_responses.empty()) {
        auto resp = m_responses.front();
        m_responses.pop();
        return resp;
    }
    return "";
}

void RealFirmwareDriver::update(double dt_seconds) {
    // Status polling is handled by FirmwareInterface's polling thread
    // Nothing to do here - positions are updated via callback
}

std::array<double, DRIVER_NUM_AXES> RealFirmwareDriver::getJointPositions() const {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    return m_positions;
}

std::string RealFirmwareDriver::getStateString() const {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    return m_lastState;
}

std::string RealFirmwareDriver::generateStatusReport() const {
    if (!m_firmware.isConnected()) {
        return "<Disconnected|MPos:0.000,0.000,0.000,0.000,0.000,0.000|FS:0,0>";
    }

    std::lock_guard<std::mutex> lock(m_cacheMutex);
    std::ostringstream ss;
    ss << "<" << m_lastState << "|MPos:";
    ss << std::fixed << std::setprecision(3);
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        if (i > 0) ss << ",";
        ss << m_positions[i];
    }
    ss << "|FS:0,0>";
    return ss.str();
}

void RealFirmwareDriver::requestStatus() {
    if (m_firmware.isConnected()) {
        m_firmware.getStatus();
    }
}

void RealFirmwareDriver::emergencyStop() {
    LOG_WARN("RealFirmwareDriver: EMERGENCY STOP!");
    if (m_firmware.isConnected()) {
        m_firmware.stopMotion();
    }
}

void RealFirmwareDriver::reset() {
    LOG_INFO("RealFirmwareDriver: Reset");
    if (m_firmware.isConnected()) {
        m_firmware.clearAlarm();
    }
}

void RealFirmwareDriver::setJointLimits(int joint, double minDeg, double maxDeg) {
    if (joint >= 0 && joint < DRIVER_NUM_AXES) {
        m_minLimits[joint] = minDeg;
        m_maxLimits[joint] = maxDeg;
    }
}

void RealFirmwareDriver::setMaxVelocity(int joint, double degPerSec) {
    if (joint >= 0 && joint < DRIVER_NUM_AXES) {
        m_maxVelocities[joint] = degPerSec;
    }
}

void RealFirmwareDriver::setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) {
    // For real hardware, send rapid jog commands for each joint
    // This is a simplified implementation - real hardware may need streaming protocol
    std::ostringstream gcode;
    gcode << "$J=G90";
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        double pos = std::clamp(positionsDeg[i], m_minLimits[i], m_maxLimits[i]);
        gcode << " J" << i << ":" << pos;
    }
    gcode << " F9000";
    sendCommand(gcode.str());
}

std::string RealFirmwareDriver::getPortName() const {
    return m_portName;
}

std::vector<std::string> RealFirmwareDriver::scanPorts() {
    return SerialPortManager::getAvailablePorts();
}

} // namespace firmware
} // namespace robot_controller
