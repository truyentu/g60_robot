#pragma once

#include "IFirmwareDriver.hpp"
#include "FirmwareInterface.hpp"
#include <memory>
#include <string>
#include <queue>
#include <mutex>

namespace robot_controller {
namespace firmware {

/**
 * Real firmware driver using USB serial to Teensy 4.1.
 * Wraps FirmwareInterface and adapts to IFirmwareDriver.
 */
class RealFirmwareDriver : public IFirmwareDriver {
public:
    RealFirmwareDriver();
    ~RealFirmwareDriver() override;

    // Connection management
    bool connect(const std::string& portName, int baudRate = 115200);
    void disconnect();
    bool autoConnect();

    // IFirmwareDriver interface
    bool isConnected() const override;
    bool sendCommand(const std::string& gcode) override;
    std::string getResponse() override;
    void update(double dt_seconds) override;
    std::array<double, DRIVER_NUM_AXES> getJointPositions() const override;
    std::string getStateString() const override;
    std::string generateStatusReport() const override;
    void requestStatus() override;
    void emergencyStop() override;
    void reset() override;
    void setJointLimits(int joint, double minDeg, double maxDeg) override;
    void setMaxVelocity(int joint, double degPerSec) override;
    void setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) override;
    std::string getDriverName() const override { return "Teensy4.1 (grblHAL)"; }
    bool isSimulation() const override { return false; }

    // Get port name
    std::string getPortName() const;

    // Scan available COM ports
    static std::vector<std::string> scanPorts();

private:
    FirmwareInterface m_firmware;
    std::string m_portName;

    // Cached positions (updated from status polling)
    mutable std::mutex m_cacheMutex;
    mutable std::array<double, DRIVER_NUM_AXES> m_positions{};
    mutable std::string m_lastState{"Idle"};
    mutable std::string m_lastStatusReport;

    // Joint limits (stored locally for reference)
    std::array<double, DRIVER_NUM_AXES> m_minLimits{-170, -190, -120, -185, -120, -350};
    std::array<double, DRIVER_NUM_AXES> m_maxLimits{170, 45, 156, 185, 120, 350};
    std::array<double, DRIVER_NUM_AXES> m_maxVelocities{156, 156, 176, 343, 384, 721};

    // Response queue
    std::queue<std::string> m_responses;
    std::mutex m_responseMutex;

    // Status polling interval tracking
    double m_statusPollAccumulator{0.0};
    static constexpr double STATUS_POLL_INTERVAL = 0.05;  // 50ms
};

} // namespace firmware
} // namespace robot_controller
