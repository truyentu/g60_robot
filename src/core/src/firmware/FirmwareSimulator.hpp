#pragma once

#include "IFirmwareDriver.hpp"
#include <array>
#include <string>
#include <queue>
#include <mutex>
#include <atomic>
#include "../jog/RuckigSmoother.hpp"

namespace robot_controller {
namespace firmware {

constexpr int SIM_NUM_AXES = 6;

class FirmwareSimulator : public IFirmwareDriver {
public:
    FirmwareSimulator();
    ~FirmwareSimulator() override = default;

    // IFirmwareDriver interface
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

private:
    bool parseGcode(const std::string& gcode);
    bool parseJogCommand(const std::string& gcode);
    void interpolateMotion(double dt);
    bool isAtTarget() const;

    enum class State { IDLE, JOG, RUN, HOLD, ALARM };
    std::atomic<State> m_state{State::IDLE};

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

    static constexpr double POSITION_TOLERANCE = 0.01;
};

} // namespace firmware
} // namespace robot_controller
