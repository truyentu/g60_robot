#pragma once

#include "IFirmwareDriver.hpp"
#include <array>
#include <string>
#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>
#include "../jog/RuckigSmoother.hpp"

namespace robot_controller {
namespace firmware {

constexpr int SIM_NUM_AXES = 6;

class FirmwareSimulator : public IFirmwareDriver {
public:
    FirmwareSimulator();
    ~FirmwareSimulator() override = default;

    // ========================================================================
    // V1 IFirmwareDriver interface (existing — unchanged signatures)
    // ========================================================================

    bool isConnected() const override { return true; }
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
    std::string getDriverName() const override { return "FirmwareSimulator"; }
    bool isSimulation() const override { return true; }

    // ========================================================================
    // V2 Structured Commands (new overrides)
    // ========================================================================

    // Drive Control
    bool enableDrives(uint8_t axisMask = 0x3F) override;
    bool disableDrives(uint8_t axisMask = 0x3F) override;
    bool resetAlarm(uint8_t axisMask = 0x3F) override;

    // Jog (structured)
    bool jogStart(uint8_t axis, int8_t direction, uint16_t speed) override;
    bool jogStop() override;

    // Motion (structured)
    bool stopMotion(uint8_t mode = 0) override;

    // Homing
    bool homeStart(uint8_t axisMask = 0x3F, uint8_t sequence = 0,
                   uint8_t method = 0) override;
    bool homeStop(uint8_t axisMask = 0x3F) override;

    // Status
    protocol::StatusPacket getStatusPacket() const override;
    uint8_t getBufferLevel() const override { return 0; }
    protocol::SystemState getSystemState() const override;

    // Unit Conversion (simulated: 2777.78 steps/degree for all axes)
    int32_t degreesToSteps(uint8_t axis, double degrees) const override;
    double stepsToDegrees(uint8_t axis, int32_t steps) const override;

    // I/O
    bool setOutput(uint8_t index, bool value) override;
    bool setOutputsBatch(uint16_t mask, uint16_t values) override;
    uint16_t getDigitalInputs() const override;
    uint16_t getDigitalOutputs() const override;

private:
    bool parseGcode(const std::string& gcode);
    bool parseJogCommand(const std::string& gcode);
    void interpolateMotion(double dt);
    bool isAtTarget() const;
    void updateHomingSimulation(double dt);

    enum class State { IDLE, JOG, RUN, HOLD, ALARM, DISABLED, HOMING };
    std::atomic<State> m_state{State::IDLE};  // Simulator: auto-enabled, start IDLE

    mutable std::mutex m_mutex;
    std::array<double, SIM_NUM_AXES> m_positions{};
    std::array<double, SIM_NUM_AXES> m_targets{};

    std::array<double, SIM_NUM_AXES> m_minLimits{-170, -190, -120, -185, -120, -350};
    std::array<double, SIM_NUM_AXES> m_maxLimits{170, 45, 156, 185, 120, 350};
    std::array<double, SIM_NUM_AXES> m_maxVelocities{156, 156, 176, 343, 384, 721};

    double m_feedRate{3600.0};  // degrees per minute
    bool m_isJogMode{false};

    std::queue<std::string> m_responses;
    bool m_statusRequested{false};

    // Ruckig S-curve trajectory smoother
    std::unique_ptr<jog::RuckigSmoother> m_smoother;
    std::array<double, SIM_NUM_AXES> m_currentVelocities{};

    // V2: Drive state (simulator auto-enables)
    bool m_drivesEnabled{true};
    uint8_t m_driveReadyMask{0x3F};   // All drives physically ready (simulated)
    uint8_t m_driveAlarmMask{0x00};

    // V2: Homing simulation
    uint8_t m_homeStatusMask{0x00};       // bit0-5: axis homed flags
    uint8_t m_homingActiveMask{0x00};     // bit0-5: axes currently homing
    double m_homingProgress[SIM_NUM_AXES]{};  // per-axis progress (0..1)
    static constexpr double HOMING_DURATION = 2.0;  // seconds per axis

    // V2: Digital I/O simulation
    uint16_t m_digitalInputs{0};
    uint16_t m_digitalOutputs{0};

    // V2: Unit conversion (simulated encoder resolution)
    std::array<double, SIM_NUM_AXES> m_stepsPerDegree{
        2777.78, 2777.78, 2777.78, 2777.78, 2777.78, 2777.78};

    static constexpr double POSITION_TOLERANCE = 0.01;
};

} // namespace firmware
} // namespace robot_controller
