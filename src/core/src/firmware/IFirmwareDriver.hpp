/**
 * @file IFirmwareDriver.hpp
 * @brief Abstract interface for firmware drivers (simulator, STM32, or plugins)
 *
 * V2 Architecture: Added structured binary protocol methods alongside
 * existing G-code interface. All new methods have default implementations
 * so existing drivers (FirmwareSimulator) compile without changes.
 */

#pragma once

#include <array>
#include <string>
#include <vector>
#include <functional>
#include "protocol/PacketTypes.hpp"
#include "protocol/PVTPoint.hpp"
#include "protocol/StatusPacket.hpp"

namespace robot_controller {
namespace firmware {

constexpr int DRIVER_NUM_AXES = 6;

/**
 * Abstract interface for firmware drivers.
 *
 * JogController, RobotController, and TrajectoryExecutor use this interface,
 * allowing seamless switching between simulator, STM32 hardware, and plugins.
 *
 * V1 (existing): G-code text-based commands
 * V2 (new):      Structured binary protocol commands
 */
class IFirmwareDriver {
public:
    virtual ~IFirmwareDriver() = default;

    // ========================================================================
    // V1 INTERFACE (existing — pure virtual, unchanged)
    // ========================================================================

    // Connection
    virtual bool isConnected() const = 0;

    // Send G-code command (legacy — used by GcodeAdapter plugin)
    virtual bool sendCommand(const std::string& gcode) = 0;

    // Get next response from firmware
    virtual std::string getResponse() = 0;

    // Update simulation/polling (call at control frequency)
    virtual void update(double dt_seconds) = 0;

    // Get current joint positions (degrees)
    virtual std::array<double, DRIVER_NUM_AXES> getJointPositions() const = 0;

    // Get state string (Idle, Run, Jog, Hold, Alarm, etc.)
    virtual std::string getStateString() const = 0;

    // Generate human-readable status report
    virtual std::string generateStatusReport() const = 0;

    // Request status update from firmware
    virtual void requestStatus() = 0;

    // Emergency stop — immediate halt
    virtual void emergencyStop() = 0;

    // Reset from alarm state
    virtual void reset() = 0;

    // Configuration
    virtual void setJointLimits(int joint, double minDeg, double maxDeg) = 0;
    virtual void setMaxVelocity(int joint, double degPerSec) = 0;

    // Direct position command (bypasses trajectory planner)
    // Used by velocity-streaming Cartesian jog for smooth continuous motion
    virtual void setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) = 0;

    // Driver info
    virtual std::string getDriverName() const = 0;
    virtual bool isSimulation() const = 0;

    // ========================================================================
    // V2 STRUCTURED COMMANDS (new — default implementations for backward compat)
    // ========================================================================

    // --- Motion ---

    /// Send a single PVT point for trajectory streaming
    virtual bool sendPVTPoint(const protocol::PVTPoint& pvt) { return false; }

    /// Send a batch of PVT points
    virtual bool sendPVTBatch(const std::vector<protocol::PVTPoint>& points) { return false; }

    /// Move to absolute position (point-to-point)
    virtual bool moveAbsolute(const std::array<int32_t, DRIVER_NUM_AXES>& stepsPos,
                              uint16_t maxSpeed, uint16_t accel) { return false; }

    /// Stop motion (0: decelerate, 1: immediate)
    virtual bool stopMotion(uint8_t mode = 0) { return false; }

    // --- Jog ---

    /// Start continuous jog on a single axis
    virtual bool jogStart(uint8_t axis, int8_t direction, uint16_t speed) { return false; }

    /// Stop jog
    virtual bool jogStop() { return false; }

    // --- Homing ---

    /// Start homing sequence
    virtual bool homeStart(uint8_t axisMask = 0x3F, uint8_t sequence = 0,
                           uint8_t method = 0) { return false; }

    /// Stop homing
    virtual bool homeStop(uint8_t axisMask = 0x3F) { return false; }

    /// Set homing parameters for a single axis
    virtual bool setHomingParams(uint8_t axis,
                                 const protocol::HomeSetParamsCmd& params) { return false; }

    // --- Drive Control ---

    /// Enable servo drives (bit mask: 0x3F = all 6 axes)
    virtual bool enableDrives(uint8_t axisMask = 0x3F) { return false; }

    /// Disable servo drives
    virtual bool disableDrives(uint8_t axisMask = 0x3F) { return false; }

    /// Reset drive alarms
    virtual bool resetAlarm(uint8_t axisMask = 0x3F) { return false; }

    // --- Status (structured) ---

    /// Get complete status packet (binary format)
    virtual protocol::StatusPacket getStatusPacket() const {
        return protocol::StatusPacket{};
    }

    /// Get PVT buffer fill level (0-255)
    virtual uint8_t getBufferLevel() const { return 0; }

    /// Get system state enum
    virtual protocol::SystemState getSystemState() const {
        return protocol::SystemState::STATE_INIT;
    }

    // --- Configuration ---

    /// Set axis parameters (steps/rev, gear ratio, limits)
    virtual bool setAxisParams(uint8_t axis,
                               const protocol::AxisParamsCmd& params) { return false; }

    // --- I/O ---

    /// Set a single digital output
    virtual bool setOutput(uint8_t index, bool value) { return false; }

    /// Set multiple digital outputs at once
    virtual bool setOutputsBatch(uint16_t mask, uint16_t values) { return false; }

    /// Get digital inputs state
    virtual uint16_t getDigitalInputs() const { return 0; }

    /// Get digital outputs state
    virtual uint16_t getDigitalOutputs() const { return 0; }

    // --- Unit Conversion ---

    /// Convert degrees to encoder steps for a given axis
    virtual int32_t degreesToSteps(uint8_t axis, double degrees) const { return 0; }

    /// Convert encoder steps to degrees for a given axis
    virtual double stepsToDegrees(uint8_t axis, int32_t steps) const { return 0.0; }

    // --- Heartbeat ---

    /// Send heartbeat to firmware (watchdog keep-alive)
    virtual bool sendHeartbeat() { return false; }
};

} // namespace firmware
} // namespace robot_controller
