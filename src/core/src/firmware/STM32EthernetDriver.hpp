/**
 * @file STM32EthernetDriver.hpp
 * @brief V2 Firmware Driver — UDP binary protocol client for STM32H743
 *
 * Communicates with STM32 via UDP:
 * - Command port (PC → STM32): binary packets
 * - Status port (STM32 → PC): 100Hz StatusPacket stream
 * - Heartbeat: 200ms keep-alive
 */

#pragma once

#include "IFirmwareDriver.hpp"
#include "protocol/BinaryProtocol.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <queue>
#include <functional>

// Forward declare Boost.Asio types to avoid header dependency in hpp
namespace boost { namespace asio {
    class io_context;
    namespace ip { class udp; }
}}

namespace robot_controller {
namespace firmware {

class STM32EthernetDriver : public IFirmwareDriver {
public:
    STM32EthernetDriver();
    ~STM32EthernetDriver() override;

    // Non-copyable
    STM32EthernetDriver(const STM32EthernetDriver&) = delete;
    STM32EthernetDriver& operator=(const STM32EthernetDriver&) = delete;

    // ========================================================================
    // Connection Management
    // ========================================================================

    bool connect(const std::string& ip = "192.168.1.100", uint16_t port = 5001);
    void disconnect();

    // ========================================================================
    // V1 IFirmwareDriver Interface
    // ========================================================================

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
    std::string getDriverName() const override { return "STM32EthernetDriver"; }
    bool isSimulation() const override { return false; }

    // ========================================================================
    // V2 Structured Commands
    // ========================================================================

    bool sendPVTPoint(const protocol::PVTPoint& pvt) override;
    bool sendPVTBatch(const std::vector<protocol::PVTPoint>& points) override;
    bool moveAbsolute(const std::array<int32_t, DRIVER_NUM_AXES>& stepsPos,
                      uint16_t maxSpeed, uint16_t accel) override;
    bool stopMotion(uint8_t mode) override;

    bool jogStart(uint8_t axis, int8_t direction, uint16_t speed) override;
    bool jogStop() override;

    bool homeStart(uint8_t axisMask, uint8_t sequence, uint8_t method) override;
    bool homeStop(uint8_t axisMask) override;
    bool setHomingParams(uint8_t axis, const protocol::HomeSetParamsCmd& params) override;

    bool enableDrives(uint8_t axisMask) override;
    bool disableDrives(uint8_t axisMask) override;
    bool resetAlarm(uint8_t axisMask) override;

    protocol::StatusPacket getStatusPacket() const override;
    uint8_t getBufferLevel() const override;
    protocol::SystemState getSystemState() const override;

    bool setAxisParams(uint8_t axis, const protocol::AxisParamsCmd& params) override;

    bool setOutput(uint8_t index, bool value) override;
    bool setOutputsBatch(uint16_t mask, uint16_t values) override;
    uint16_t getDigitalInputs() const override;
    uint16_t getDigitalOutputs() const override;

    int32_t degreesToSteps(uint8_t axis, double degrees) const override;
    double stepsToDegrees(uint8_t axis, int32_t steps) const override;

    bool sendHeartbeat() override;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setStepsPerDegree(uint8_t axis, double stepsPerDeg);
    std::string getRemoteIp() const;
    uint16_t getRemotePort() const;

    // Callback for async events
    using AlarmCallback = std::function<void(const protocol::AlarmPacket&)>;
    using HomeCompleteCallback = std::function<void(const protocol::HomeCompletePacket&)>;
    using PacketLogCallback = std::function<void(const std::string& direction,
        uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len)>;

    void setAlarmCallback(AlarmCallback cb);
    void setHomeCompleteCallback(HomeCompleteCallback cb);
    void setPacketLogCallback(PacketLogCallback cb);

private:
    // Network I/O (implementation in .cpp with Boost.Asio)
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    // State
    mutable std::mutex m_statusMutex;
    protocol::StatusPacket m_lastStatus{};
    std::atomic<bool> m_connected{false};
    uint8_t m_txSeq{0};

    std::string m_remoteIp;
    uint16_t m_remotePort{5001};

    // Unit conversion
    std::array<double, DRIVER_NUM_AXES> m_stepsPerDegree{
        2777.78, 2777.78, 2777.78, 2777.78, 2777.78, 2777.78};

    // Joint limits (for V1 compat)
    std::array<double, DRIVER_NUM_AXES> m_minLimits{-170, -190, -120, -185, -120, -350};
    std::array<double, DRIVER_NUM_AXES> m_maxLimits{170, 45, 156, 185, 120, 350};
    std::array<double, DRIVER_NUM_AXES> m_maxVelocities{156, 156, 176, 343, 384, 721};

    // Response queue (for V1 compat)
    std::queue<std::string> m_responses;
    mutable std::mutex m_responseMutex;

    // Callbacks
    AlarmCallback m_alarmCallback;
    HomeCompleteCallback m_homeCompleteCallback;
    PacketLogCallback m_packetLogCallback;

    // Helpers
    bool sendPacket(protocol::CommandType type,
                    const uint8_t* payload = nullptr, uint16_t payloadLen = 0);
    void processReceivedPacket(const uint8_t* data, size_t len);
};

} // namespace firmware
} // namespace robot_controller
