#pragma once

#include <array>
#include <string>
#include <functional>

namespace robot_controller {
namespace firmware {

constexpr int DRIVER_NUM_AXES = 6;

/**
 * Abstract interface for firmware drivers (simulator or real hardware).
 * JogController and RobotController use this interface,
 * allowing seamless switching between sim and real.
 */
class IFirmwareDriver {
public:
    virtual ~IFirmwareDriver() = default;

    // Connection
    virtual bool isConnected() const = 0;

    // Send G-code command
    virtual bool sendCommand(const std::string& gcode) = 0;

    // Get next response from firmware
    virtual std::string getResponse() = 0;

    // Update simulation/polling (call at control frequency)
    virtual void update(double dt_seconds) = 0;

    // Get current joint positions (degrees)
    virtual std::array<double, DRIVER_NUM_AXES> getJointPositions() const = 0;

    // Get state string (Idle, Run, Jog, Hold, Alarm)
    virtual std::string getStateString() const = 0;

    // Generate status report
    virtual std::string generateStatusReport() const = 0;

    // Request status
    virtual void requestStatus() = 0;

    // Emergency stop
    virtual void emergencyStop() = 0;

    // Reset from alarm
    virtual void reset() = 0;

    // Configuration
    virtual void setJointLimits(int joint, double minDeg, double maxDeg) = 0;
    virtual void setMaxVelocity(int joint, double degPerSec) = 0;

    // Direct position command (bypasses G-code/trajectory planner)
    // Used by velocity-streaming Cartesian jog for smooth continuous motion
    virtual void setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) = 0;

    // Driver info
    virtual std::string getDriverName() const = 0;
    virtual bool isSimulation() const = 0;
};

} // namespace firmware
} // namespace robot_controller
