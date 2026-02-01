#pragma once

#include "WeldingTypes.hpp"
#include <functional>
#include <cstdint>

namespace robotics {
namespace welding {

/**
 * Interface for welding I/O hardware
 */
class IWeldingIO {
public:
    virtual ~IWeldingIO() = default;

    // ========================================================================
    // Outputs
    // ========================================================================

    virtual void setGasValve(bool on) = 0;
    virtual void setWireFeed(bool on) = 0;
    virtual void setArcEnable(bool on) = 0;
    virtual void setTorchTrigger(bool on) = 0;
    virtual void setRobotReady(bool ready) = 0;

    virtual void setAllOutputs(const WeldingOutputs& outputs) = 0;

    // ========================================================================
    // Inputs
    // ========================================================================

    virtual WeldingInputs readInputs() = 0;

    virtual bool getArcDetect() = 0;
    virtual bool getGasFlowOk() = 0;
    virtual bool getWireOk() = 0;
    virtual bool getPowerSourceReady() = 0;
    virtual bool getPowerSourceFault() = 0;

    // ========================================================================
    // Analog
    // ========================================================================

    virtual float readCurrent() = 0;
    virtual float readVoltage() = 0;
    virtual float readWireSpeed() = 0;

    virtual void setCurrentCommand(float current) = 0;
    virtual void setVoltageCommand(float voltage) = 0;
    virtual void setWireSpeedCommand(float speed) = 0;
};

/**
 * Simulated welding I/O for testing
 */
class SimulatedWeldingIO : public IWeldingIO {
public:
    SimulatedWeldingIO();

    // Outputs
    void setGasValve(bool on) override;
    void setWireFeed(bool on) override;
    void setArcEnable(bool on) override;
    void setTorchTrigger(bool on) override;
    void setRobotReady(bool ready) override;
    void setAllOutputs(const WeldingOutputs& outputs) override;

    // Inputs
    WeldingInputs readInputs() override;
    bool getArcDetect() override;
    bool getGasFlowOk() override;
    bool getWireOk() override;
    bool getPowerSourceReady() override;
    bool getPowerSourceFault() override;

    // Analog
    float readCurrent() override;
    float readVoltage() override;
    float readWireSpeed() override;
    void setCurrentCommand(float current) override;
    void setVoltageCommand(float voltage) override;
    void setWireSpeedCommand(float speed) override;

    // Simulation control
    void setSimulatedInputs(const WeldingInputs& inputs) { simInputs_ = inputs; }
    void setSimulatedFeedback(float current, float voltage, float wireSpeed);
    WeldingOutputs getOutputs() const { return outputs_; }

private:
    WeldingOutputs outputs_;
    WeldingInputs simInputs_;

    float commandedCurrent_;
    float commandedVoltage_;
    float commandedWireSpeed_;

    float simulatedCurrent_;
    float simulatedVoltage_;
    float simulatedWireSpeed_;
};

// Implementation
inline SimulatedWeldingIO::SimulatedWeldingIO()
    : commandedCurrent_(0), commandedVoltage_(0), commandedWireSpeed_(0),
      simulatedCurrent_(0), simulatedVoltage_(0), simulatedWireSpeed_(0) {

    simInputs_.powerSourceReady = true;
    simInputs_.gasFlowOk = true;
    simInputs_.wireOk = true;
}

inline void SimulatedWeldingIO::setGasValve(bool on) { outputs_.gasValve = on; }
inline void SimulatedWeldingIO::setWireFeed(bool on) { outputs_.wireFeed = on; }
inline void SimulatedWeldingIO::setArcEnable(bool on) { outputs_.arcEnable = on; }
inline void SimulatedWeldingIO::setTorchTrigger(bool on) { outputs_.torchTrigger = on; }
inline void SimulatedWeldingIO::setRobotReady(bool ready) { outputs_.robotReady = ready; }

inline void SimulatedWeldingIO::setAllOutputs(const WeldingOutputs& outputs) {
    outputs_ = outputs;
}

inline WeldingInputs SimulatedWeldingIO::readInputs() {
    // Simulate arc detection when arc is enabled and wire is feeding
    if (outputs_.arcEnable && outputs_.wireFeed && outputs_.torchTrigger) {
        simInputs_.arcDetect = true;
        simInputs_.currentFlowing = true;
    } else {
        simInputs_.arcDetect = false;
        simInputs_.currentFlowing = false;
    }

    return simInputs_;
}

inline bool SimulatedWeldingIO::getArcDetect() { return simInputs_.arcDetect; }
inline bool SimulatedWeldingIO::getGasFlowOk() { return simInputs_.gasFlowOk; }
inline bool SimulatedWeldingIO::getWireOk() { return simInputs_.wireOk; }
inline bool SimulatedWeldingIO::getPowerSourceReady() { return simInputs_.powerSourceReady; }
inline bool SimulatedWeldingIO::getPowerSourceFault() { return simInputs_.powerSourceFault; }

inline float SimulatedWeldingIO::readCurrent() {
    if (outputs_.arcEnable && outputs_.torchTrigger) {
        return commandedCurrent_ * 0.95f;  // Simulate 95% of commanded
    }
    return 0;
}

inline float SimulatedWeldingIO::readVoltage() {
    if (outputs_.arcEnable && outputs_.torchTrigger) {
        return commandedVoltage_ * 0.98f;
    }
    return 0;
}

inline float SimulatedWeldingIO::readWireSpeed() {
    if (outputs_.wireFeed) {
        return commandedWireSpeed_;
    }
    return 0;
}

inline void SimulatedWeldingIO::setCurrentCommand(float current) {
    commandedCurrent_ = current;
}

inline void SimulatedWeldingIO::setVoltageCommand(float voltage) {
    commandedVoltage_ = voltage;
}

inline void SimulatedWeldingIO::setWireSpeedCommand(float speed) {
    commandedWireSpeed_ = speed;
}

inline void SimulatedWeldingIO::setSimulatedFeedback(
    float current, float voltage, float wireSpeed) {
    simulatedCurrent_ = current;
    simulatedVoltage_ = voltage;
    simulatedWireSpeed_ = wireSpeed;
}

} // namespace welding
} // namespace robotics
