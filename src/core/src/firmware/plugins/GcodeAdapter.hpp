/**
 * @file GcodeAdapter.hpp
 * @brief Adapter plugin that translates G-code text commands to V2 structured calls
 *
 * Enables backward compatibility: code that calls sendCommand("G1 X10 Y20 F3600")
 * gets translated to the appropriate V2 moveAbsolute/jogStart/homeStart etc.
 *
 * Usage:
 *   auto adapter = std::make_shared<GcodeAdapter>(realDriver);
 *   jogController->setFirmwareDriver(adapter);
 *   // Now jogController can use G-code or structured API
 */

#pragma once

#include "../IFirmwareDriver.hpp"
#include "../LegacyGrblProtocol.hpp"
#include <memory>
#include <string>
#include <queue>
#include <mutex>

namespace robot_controller {
namespace firmware {
namespace plugins {

class GcodeAdapter : public IFirmwareDriver {
public:
    explicit GcodeAdapter(std::shared_ptr<IFirmwareDriver> innerDriver);
    ~GcodeAdapter() override = default;

    // ========================================================================
    // V1 Interface — Translates G-code to V2 calls on inner driver
    // ========================================================================

    bool isConnected() const override { return m_inner->isConnected(); }

    bool sendCommand(const std::string& gcode) override;

    std::string getResponse() override;
    void update(double dt_seconds) override { m_inner->update(dt_seconds); }

    std::array<double, DRIVER_NUM_AXES> getJointPositions() const override {
        return m_inner->getJointPositions();
    }

    std::string getStateString() const override { return m_inner->getStateString(); }
    std::string generateStatusReport() const override { return m_inner->generateStatusReport(); }
    void requestStatus() override { m_inner->requestStatus(); }
    void emergencyStop() override { m_inner->emergencyStop(); }
    void reset() override { m_inner->reset(); }
    void setJointLimits(int joint, double minDeg, double maxDeg) override {
        m_inner->setJointLimits(joint, minDeg, maxDeg);
    }
    void setMaxVelocity(int joint, double degPerSec) override {
        m_inner->setMaxVelocity(joint, degPerSec);
    }
    void setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& pos) override {
        m_inner->setJointPositionsDirect(pos);
    }
    std::string getDriverName() const override {
        return "GcodeAdapter(" + m_inner->getDriverName() + ")";
    }
    bool isSimulation() const override { return m_inner->isSimulation(); }

    // ========================================================================
    // V2 Interface — Pass through to inner driver
    // ========================================================================

    bool sendPVTPoint(const protocol::PVTPoint& pvt) override {
        return m_inner->sendPVTPoint(pvt);
    }
    bool sendPVTBatch(const std::vector<protocol::PVTPoint>& pts) override {
        return m_inner->sendPVTBatch(pts);
    }
    bool moveAbsolute(const std::array<int32_t, DRIVER_NUM_AXES>& s,
                      uint16_t ms, uint16_t a) override {
        return m_inner->moveAbsolute(s, ms, a);
    }
    bool stopMotion(uint8_t mode) override { return m_inner->stopMotion(mode); }
    bool jogStart(uint8_t axis, int8_t dir, uint16_t speed) override {
        return m_inner->jogStart(axis, dir, speed);
    }
    bool jogStop() override { return m_inner->jogStop(); }
    bool homeStart(uint8_t mask, uint8_t seq, uint8_t method) override {
        return m_inner->homeStart(mask, seq, method);
    }
    bool homeStop(uint8_t mask) override { return m_inner->homeStop(mask); }
    bool setHomingParams(uint8_t axis, const protocol::HomeSetParamsCmd& p) override {
        return m_inner->setHomingParams(axis, p);
    }
    bool enableDrives(uint8_t mask) override { return m_inner->enableDrives(mask); }
    bool disableDrives(uint8_t mask) override { return m_inner->disableDrives(mask); }
    bool resetAlarm(uint8_t mask) override { return m_inner->resetAlarm(mask); }
    protocol::StatusPacket getStatusPacket() const override {
        return m_inner->getStatusPacket();
    }
    uint8_t getBufferLevel() const override { return m_inner->getBufferLevel(); }
    protocol::SystemState getSystemState() const override {
        return m_inner->getSystemState();
    }
    bool setAxisParams(uint8_t axis, const protocol::AxisParamsCmd& p) override {
        return m_inner->setAxisParams(axis, p);
    }
    bool setOutput(uint8_t index, bool value) override {
        return m_inner->setOutput(index, value);
    }
    bool setOutputsBatch(uint16_t mask, uint16_t values) override {
        return m_inner->setOutputsBatch(mask, values);
    }
    uint16_t getDigitalInputs() const override { return m_inner->getDigitalInputs(); }
    uint16_t getDigitalOutputs() const override { return m_inner->getDigitalOutputs(); }
    int32_t degreesToSteps(uint8_t axis, double deg) const override {
        return m_inner->degreesToSteps(axis, deg);
    }
    double stepsToDegrees(uint8_t axis, int32_t steps) const override {
        return m_inner->stepsToDegrees(axis, steps);
    }
    bool sendHeartbeat() override { return m_inner->sendHeartbeat(); }

private:
    std::shared_ptr<IFirmwareDriver> m_inner;

    // Translation helpers
    bool translateMoveCommand(const std::string& gcode);
    bool translateJogCommand(const std::string& gcode);
    bool translateHomingCommand(const std::string& gcode);
    bool translateRealTimeChar(char c);

    // Response queue for translated commands
    std::queue<std::string> m_responses;
    std::mutex m_responseMutex;

    // Conversion
    std::array<double, DRIVER_NUM_AXES> m_stepsPerDegree{
        2777.78, 2777.78, 2777.78, 2777.78, 2777.78, 2777.78};

    double m_feedRate{3600.0};
};

} // namespace plugins
} // namespace firmware
} // namespace robot_controller
