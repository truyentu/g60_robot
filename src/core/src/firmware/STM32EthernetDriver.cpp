/**
 * @file STM32EthernetDriver.cpp
 * @brief V2 Firmware Driver — UDP binary protocol implementation
 *
 * Uses Boost.Asio for async UDP communication.
 * Command path: serializePacket → sendto
 * Status path: recvfrom → parsePacket → update m_lastStatus
 */

#include "STM32EthernetDriver.hpp"
#include "../logging/Logger.hpp"
#include <boost/asio.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace robot_controller {
namespace firmware {

using boost::asio::ip::udp;
using namespace protocol;

// ============================================================================
// Impl (pimpl — owns Boost.Asio sockets and threads)
// ============================================================================

struct STM32EthernetDriver::Impl {
    boost::asio::io_context ioContext;
    std::unique_ptr<udp::socket> cmdSocket;    // Send commands
    std::unique_ptr<udp::socket> statusSocket; // Receive status
    udp::endpoint remoteEndpoint;

    std::thread ioThread;
    std::thread heartbeatThread;
    std::atomic<bool> running{false};

    // Receive buffer
    static constexpr size_t RX_BUF_SIZE = 1024;
    uint8_t rxBuffer[RX_BUF_SIZE];
    udp::endpoint senderEndpoint;

    // Heartbeat
    std::chrono::steady_clock::time_point lastHeartbeat;
    static constexpr int HEARTBEAT_MS = 200;

    void startAsyncReceive(STM32EthernetDriver* driver) {
        if (!statusSocket || !running) return;

        statusSocket->async_receive_from(
            boost::asio::buffer(rxBuffer, RX_BUF_SIZE),
            senderEndpoint,
            [this, driver](const boost::system::error_code& ec, size_t bytesReceived) {
                if (!ec && bytesReceived > 0) {
                    driver->processReceivedPacket(rxBuffer, bytesReceived);
                }
                if (running) {
                    startAsyncReceive(driver);
                }
            });
    }
};

// ============================================================================
// Constructor / Destructor
// ============================================================================

STM32EthernetDriver::STM32EthernetDriver()
    : m_impl(std::make_unique<Impl>()) {
    m_lastStatus.clear();
    LOG_INFO("STM32EthernetDriver created");
}

STM32EthernetDriver::~STM32EthernetDriver() {
    disconnect();
}

// ============================================================================
// Connection
// ============================================================================

bool STM32EthernetDriver::connect(const std::string& ip, uint16_t port) {
    if (m_connected) {
        disconnect();
    }

    m_remoteIp = ip;
    m_remotePort = port;

    try {
        // Resolve remote endpoint
        m_impl->remoteEndpoint = udp::endpoint(
            boost::asio::ip::make_address(ip), port);

        // Command socket (ephemeral local port)
        m_impl->cmdSocket = std::make_unique<udp::socket>(
            m_impl->ioContext, udp::endpoint(udp::v4(), 0));

        // Status socket (fixed local port for receiving)
        m_impl->statusSocket = std::make_unique<udp::socket>(
            m_impl->ioContext, udp::endpoint(udp::v4(), port + 1));

        m_impl->running = true;

        // Start async receive
        m_impl->startAsyncReceive(this);

        // IO thread
        m_impl->ioThread = std::thread([this]() {
            try {
                m_impl->ioContext.run();
            } catch (const std::exception& e) {
                LOG_ERROR("STM32EthernetDriver IO error: {}", e.what());
            }
        });

        // Heartbeat thread
        m_impl->heartbeatThread = std::thread([this]() {
            while (m_impl->running) {
                sendHeartbeat();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(Impl::HEARTBEAT_MS));
            }
        });

        m_connected = true;
        LOG_INFO("STM32EthernetDriver: Connected to {}:{}", ip, port);
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("STM32EthernetDriver: Connection failed: {}", e.what());
        m_impl->running = false;
        return false;
    }
}

void STM32EthernetDriver::disconnect() {
    if (!m_connected) return;

    m_impl->running = false;
    m_connected = false;

    // Stop IO context
    m_impl->ioContext.stop();

    if (m_impl->ioThread.joinable()) {
        m_impl->ioThread.join();
    }
    if (m_impl->heartbeatThread.joinable()) {
        m_impl->heartbeatThread.join();
    }

    // Close sockets
    if (m_impl->cmdSocket && m_impl->cmdSocket->is_open()) {
        boost::system::error_code ec;
        m_impl->cmdSocket->close(ec);
    }
    if (m_impl->statusSocket && m_impl->statusSocket->is_open()) {
        boost::system::error_code ec;
        m_impl->statusSocket->close(ec);
    }

    m_impl->cmdSocket.reset();
    m_impl->statusSocket.reset();

    // Reset IO context for potential reconnect
    m_impl->ioContext.restart();

    LOG_INFO("STM32EthernetDriver: Disconnected");
}

// ============================================================================
// Send helper
// ============================================================================

bool STM32EthernetDriver::sendPacket(CommandType type,
                                      const uint8_t* payload, uint16_t payloadLen) {
    if (!m_connected || !m_impl->cmdSocket) return false;

    auto packet = serializePacket(m_txSeq++, static_cast<uint8_t>(type),
                                   payload, payloadLen);
    if (packet.empty()) return false;

    try {
        m_impl->cmdSocket->send_to(
            boost::asio::buffer(packet), m_impl->remoteEndpoint);

        if (m_packetLogCallback) {
            try {
                m_packetLogCallback("TX", static_cast<uint8_t>(type),
                    static_cast<uint8_t>(m_txSeq - 1), payload, payloadLen);
            } catch (...) {
                // Don't let callback exceptions affect send
            }
        }

        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("STM32EthernetDriver: Send failed: {}", e.what());
        return false;
    }
}

// ============================================================================
// Receive / Parse
// ============================================================================

void STM32EthernetDriver::processReceivedPacket(const uint8_t* data, size_t len) {
    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    if (!parsePacket(data, len, header, &payloadData, &payloadLen)) {
        return;
    }

    // Log received packet (rate-limit RSP_STATUS: only every 50th)
    if (m_packetLogCallback) {
        try {
            static int rxStatusSkip = 0;
            bool isStatus = (header.type == static_cast<uint8_t>(ResponseType::RSP_STATUS));
            if (!isStatus || (++rxStatusSkip % 50 == 0)) {
                m_packetLogCallback("RX", header.type, header.seq, payloadData, payloadLen);
            }
        } catch (...) {
            // Don't let callback exceptions kill the receive loop
        }
    }

    auto rspType = static_cast<ResponseType>(header.type);

    switch (rspType) {
        case ResponseType::RSP_STATUS: {
            StatusPacket status;
            if (extractPayload(payloadData, payloadLen, status)) {
                std::lock_guard<std::mutex> lock(m_statusMutex);
                m_lastStatus = status;
            }
            break;
        }
        case ResponseType::RSP_ALARM: {
            AlarmPacket alarm;
            if (extractPayload(payloadData, payloadLen, alarm)) {
                LOG_WARN("STM32: Alarm axis={} code={}", alarm.axis, alarm.alarm_code);
                if (m_alarmCallback) {
                    m_alarmCallback(alarm);
                }
            }
            break;
        }
        case ResponseType::RSP_HOME_COMPLETE: {
            HomeCompletePacket hc;
            if (extractPayload(payloadData, payloadLen, hc)) {
                LOG_INFO("STM32: Home complete axis={} success={}", hc.axis, hc.success);
                if (m_homeCompleteCallback) {
                    m_homeCompleteCallback(hc);
                }
            }
            break;
        }
        case ResponseType::RSP_ACK: {
            AckPacket ack;
            if (extractPayload(payloadData, payloadLen, ack)) {
                if (ack.error_code != 0) {
                    LOG_WARN("STM32: NACK cmd=0x{:02X} error={}", ack.cmd_type, ack.error_code);
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// V1 Interface Implementation
// ============================================================================

bool STM32EthernetDriver::isConnected() const {
    return m_connected.load();
}

bool STM32EthernetDriver::sendCommand(const std::string& gcode) {
    // V1 compat: G-code not supported on STM32 binary protocol
    LOG_WARN("STM32EthernetDriver: sendCommand('{}') — G-code not supported, use structured API", gcode);
    return false;
}

std::string STM32EthernetDriver::getResponse() {
    std::lock_guard<std::mutex> lock(m_responseMutex);
    if (!m_responses.empty()) {
        auto resp = m_responses.front();
        m_responses.pop();
        return resp;
    }
    return "";
}

void STM32EthernetDriver::update(double dt_seconds) {
    // Status is received asynchronously; nothing to poll
}

std::array<double, DRIVER_NUM_AXES> STM32EthernetDriver::getJointPositions() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    std::array<double, DRIVER_NUM_AXES> pos{};
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        pos[i] = static_cast<double>(m_lastStatus.actual_pos[i]) / m_stepsPerDegree[i];
    }
    return pos;
}

std::string STM32EthernetDriver::getStateString() const {
    return systemStateToString(getSystemState());
}

std::string STM32EthernetDriver::generateStatusReport() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    std::ostringstream ss;
    ss << "<" << getStateString() << "|MPos:";
    ss << std::fixed << std::setprecision(3);
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        if (i > 0) ss << ",";
        double deg = static_cast<double>(m_lastStatus.actual_pos[i]) / m_stepsPerDegree[i];
        ss << deg;
    }
    ss << "|DrvReady:0x" << std::hex << static_cast<int>(m_lastStatus.drive_ready)
       << "|DrvAlarm:0x" << std::hex << static_cast<int>(m_lastStatus.drive_alarm)
       << "|Homed:0x" << std::hex << static_cast<int>(m_lastStatus.home_status)
       << "|Buf:" << std::dec << static_cast<int>(m_lastStatus.pvt_buffer_lvl)
       << ">";
    return ss.str();
}

void STM32EthernetDriver::requestStatus() {
    // Status is sent automatically at 100Hz by STM32; no request needed
}

void STM32EthernetDriver::emergencyStop() {
    LOG_WARN("STM32EthernetDriver: E-STOP");
    sendPacket(CommandType::CMD_E_STOP);
}

void STM32EthernetDriver::reset() {
    resetAlarm(0x3F);
}

void STM32EthernetDriver::setJointLimits(int joint, double minDeg, double maxDeg) {
    if (joint >= 0 && joint < DRIVER_NUM_AXES) {
        m_minLimits[joint] = minDeg;
        m_maxLimits[joint] = maxDeg;
    }
}

void STM32EthernetDriver::setMaxVelocity(int joint, double degPerSec) {
    if (joint >= 0 && joint < DRIVER_NUM_AXES) {
        m_maxVelocities[joint] = degPerSec;
    }
}

void STM32EthernetDriver::setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) {
    // Convert to steps and send as MoveAbsolute
    std::array<int32_t, DRIVER_NUM_AXES> steps{};
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        double deg = std::clamp(positionsDeg[i], m_minLimits[i], m_maxLimits[i]);
        steps[i] = static_cast<int32_t>(deg * m_stepsPerDegree[i]);
    }
    moveAbsolute(steps, 1000, 500);
}

// ============================================================================
// V2 Structured Commands
// ============================================================================

bool STM32EthernetDriver::sendPVTPoint(const PVTPoint& pvt) {
    return sendPacket(CommandType::CMD_PVT_POINT,
                      reinterpret_cast<const uint8_t*>(&pvt), sizeof(PVTPoint));
}

bool STM32EthernetDriver::sendPVTBatch(const std::vector<PVTPoint>& points) {
    if (points.empty()) return true;
    // Send as single CMD_PVT_BATCH with all points concatenated
    size_t totalSize = points.size() * sizeof(PVTPoint);
    if (totalSize > MAX_PAYLOAD) {
        // Split into multiple batches
        for (const auto& pt : points) {
            if (!sendPVTPoint(pt)) return false;
        }
        return true;
    }
    return sendPacket(CommandType::CMD_PVT_BATCH,
                      reinterpret_cast<const uint8_t*>(points.data()),
                      static_cast<uint16_t>(totalSize));
}

bool STM32EthernetDriver::moveAbsolute(const std::array<int32_t, DRIVER_NUM_AXES>& stepsPos,
                                         uint16_t maxSpeed, uint16_t accel) {
    MoveAbsoluteCmd cmd{};
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        cmd.position[i] = stepsPos[i];
    }
    cmd.max_speed = maxSpeed;
    cmd.accel = accel;
    return sendPacket(CommandType::CMD_MOVE_ABSOLUTE,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::stopMotion(uint8_t mode) {
    StopMotionCmd cmd{mode};
    return sendPacket(CommandType::CMD_STOP_MOTION,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::jogStart(uint8_t axis, int8_t direction, uint16_t speed) {
    JogStartCmd cmd{axis, direction, speed};
    return sendPacket(CommandType::CMD_JOG_START,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::jogStop() {
    return sendPacket(CommandType::CMD_JOG_STOP);
}

bool STM32EthernetDriver::homeStart(uint8_t axisMask, uint8_t sequence, uint8_t method) {
    HomeStartCmd cmd{axisMask, sequence, method};
    return sendPacket(CommandType::CMD_HOME_START,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::homeStop(uint8_t axisMask) {
    HomeStopCmd cmd{axisMask};
    return sendPacket(CommandType::CMD_HOME_STOP,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::setHomingParams(uint8_t axis, const HomeSetParamsCmd& params) {
    return sendPacket(CommandType::CMD_HOME_SET_PARAMS,
                      reinterpret_cast<const uint8_t*>(&params), sizeof(params));
}

bool STM32EthernetDriver::enableDrives(uint8_t axisMask) {
    DriveControlCmd cmd{axisMask};
    return sendPacket(CommandType::CMD_ENABLE_DRIVES,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::disableDrives(uint8_t axisMask) {
    DriveControlCmd cmd{axisMask};
    return sendPacket(CommandType::CMD_DISABLE_DRIVES,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::resetAlarm(uint8_t axisMask) {
    DriveControlCmd cmd{axisMask};
    return sendPacket(CommandType::CMD_RESET_ALARM,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

StatusPacket STM32EthernetDriver::getStatusPacket() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_lastStatus;
}

uint8_t STM32EthernetDriver::getBufferLevel() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_lastStatus.pvt_buffer_lvl;
}

SystemState STM32EthernetDriver::getSystemState() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return static_cast<SystemState>(m_lastStatus.state);
}

bool STM32EthernetDriver::setAxisParams(uint8_t axis, const AxisParamsCmd& params) {
    return sendPacket(CommandType::CMD_SET_AXIS_PARAMS,
                      reinterpret_cast<const uint8_t*>(&params), sizeof(params));
}

bool STM32EthernetDriver::setOutput(uint8_t index, bool value) {
    SetOutputCmd cmd{index, static_cast<uint8_t>(value ? 1 : 0)};
    return sendPacket(CommandType::CMD_SET_OUTPUT,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

bool STM32EthernetDriver::setOutputsBatch(uint16_t mask, uint16_t values) {
    SetOutputsBatchCmd cmd{mask, values};
    return sendPacket(CommandType::CMD_SET_OUTPUTS_BATCH,
                      reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

uint16_t STM32EthernetDriver::getDigitalInputs() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_lastStatus.digital_inputs;
}

uint16_t STM32EthernetDriver::getDigitalOutputs() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_lastStatus.digital_outputs;
}

int32_t STM32EthernetDriver::degreesToSteps(uint8_t axis, double degrees) const {
    if (axis >= DRIVER_NUM_AXES) return 0;
    return static_cast<int32_t>(degrees * m_stepsPerDegree[axis]);
}

double STM32EthernetDriver::stepsToDegrees(uint8_t axis, int32_t steps) const {
    if (axis >= DRIVER_NUM_AXES) return 0.0;
    return static_cast<double>(steps) / m_stepsPerDegree[axis];
}

bool STM32EthernetDriver::sendHeartbeat() {
    return sendPacket(CommandType::CMD_HEARTBEAT);
}

// ============================================================================
// Configuration
// ============================================================================

void STM32EthernetDriver::setStepsPerDegree(uint8_t axis, double stepsPerDeg) {
    if (axis < DRIVER_NUM_AXES) {
        m_stepsPerDegree[axis] = stepsPerDeg;
    }
}

std::string STM32EthernetDriver::getRemoteIp() const {
    return m_remoteIp;
}

uint16_t STM32EthernetDriver::getRemotePort() const {
    return m_remotePort;
}

void STM32EthernetDriver::setAlarmCallback(AlarmCallback cb) {
    m_alarmCallback = std::move(cb);
}

void STM32EthernetDriver::setHomeCompleteCallback(HomeCompleteCallback cb) {
    m_homeCompleteCallback = std::move(cb);
}

void STM32EthernetDriver::setPacketLogCallback(PacketLogCallback cb) {
    m_packetLogCallback = std::move(cb);
}

} // namespace firmware
} // namespace robot_controller
