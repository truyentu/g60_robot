/**
 * @file RobotController.cpp
 * @brief Robot controller implementation
 */

#include "RobotController.hpp"
#include "../logging/Logger.hpp"
#include <chrono>
#include <algorithm>

namespace robot_controller {

using namespace state;
using namespace config;
using namespace ipc;

RobotController::RobotController()
    : m_stateMachine(std::make_unique<StateMachine>())
{
    LOG_DEBUG("RobotController created");
}

RobotController::~RobotController() {
    stop();
}

bool RobotController::initialize(const std::string& configDir) {
    LOG_INFO("Initializing RobotController...");

    // Load configuration
    auto& config = ConfigManager::instance();
    if (!config.loadAll(configDir)) {
        LOG_ERROR("Failed to load configuration");
        return false;
    }

    const auto& sysConfig = config.systemConfig();

    // Store cycle time
    m_cycleTimeMs = sysConfig.control.cycle_time_ms;
    m_statusPublishHz = sysConfig.control.status_publish_hz;

    // Create IPC server
    std::string repAddr = "tcp://" + sysConfig.ipc.bind_address + ":" +
                          std::to_string(sysConfig.ipc.rep_port);
    std::string pubAddr = "tcp://" + sysConfig.ipc.bind_address + ":" +
                          std::to_string(sysConfig.ipc.pub_port);

    m_ipcServer = std::make_unique<IpcServer>(repAddr, pubAddr);
    registerIpcHandlers();

    // Setup state machine callbacks
    m_stateMachine->setStateChangeCallback(
        [this](RobotState oldState, RobotState newState) {
            LOG_INFO("State changed: {} -> {}", toString(oldState), toString(newState));
            updateStatus();
        });

    m_stateMachine->setErrorCallback(
        [this](ErrorCode code, const std::string& msg) {
            LOG_ERROR("Error: {} - {}", toString(code), msg);
            updateStatus();
        });

    // Initialize complete
    m_stateMachine->processEvent(StateEvent::INIT_COMPLETE);

    LOG_INFO("RobotController initialized");
    return true;
}

bool RobotController::start() {
    if (m_running) {
        LOG_WARN("RobotController already running");
        return true;
    }

    // Start IPC server
    if (!m_ipcServer->start()) {
        LOG_ERROR("Failed to start IPC server");
        return false;
    }

    // Start control loop
    m_running = true;
    m_controlThread = std::thread(&RobotController::controlLoop, this);

    LOG_INFO("RobotController started");
    return true;
}

void RobotController::stop() {
    if (!m_running) {
        return;
    }

    LOG_INFO("Stopping RobotController...");

    m_running = false;

    if (m_controlThread.joinable()) {
        m_controlThread.join();
    }

    if (m_ipcServer) {
        m_ipcServer->stop();
    }

    LOG_INFO("RobotController stopped");
}

RobotStatus RobotController::getStatus() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_status;
}

bool RobotController::enable() {
    return m_stateMachine->enable();
}

bool RobotController::disable() {
    return m_stateMachine->disable();
}

bool RobotController::reset() {
    return m_stateMachine->reset();
}

bool RobotController::home() {
    if (!m_stateMachine->isEnabled()) {
        LOG_WARN("Cannot home: robot not enabled");
        return false;
    }

    if (m_stateMachine->processEvent(StateEvent::HOME_REQUEST)) {
        LOG_INFO("Homing started");

        // TODO: Actual homing implementation in Phase 2
        // For now, simulate homing complete
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            m_stateMachine->processEvent(StateEvent::HOME_COMPLETE);
            LOG_INFO("Homing complete (simulated)");
        }).detach();

        return true;
    }

    return false;
}

bool RobotController::setMode(RobotMode mode) {
    return m_stateMachine->setMode(mode);
}

bool RobotController::setSpeedOverride(double percent) {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    m_status.speedOverride = std::clamp(percent, 0.0, 100.0);
    LOG_DEBUG("Speed override set to {}%", m_status.speedOverride);
    return true;
}

bool RobotController::jogJoint(int joint, double speed) {
    if (!m_stateMachine->canJog()) {
        LOG_WARN("Cannot jog: not in jog-able state");
        return false;
    }

    // TODO: Implement in Phase 2
    LOG_DEBUG("Jog joint {} at speed {}", joint, speed);
    return true;
}

bool RobotController::jogCartesian(int axis, double speed) {
    if (!m_stateMachine->canJog()) {
        LOG_WARN("Cannot jog: not in jog-able state");
        return false;
    }

    // TODO: Implement in Phase 2
    LOG_DEBUG("Jog Cartesian axis {} at speed {}", axis, speed);
    return true;
}

bool RobotController::stopJog() {
    // TODO: Implement in Phase 2
    LOG_DEBUG("Stop jog");
    return true;
}

void RobotController::controlLoop() {
    LOG_DEBUG("Control loop started, cycle time: {}ms", m_cycleTimeMs);

    auto lastStatusPublish = std::chrono::steady_clock::now();
    int statusIntervalMs = 1000 / m_statusPublishHz;

    while (m_running) {
        auto cycleStart = std::chrono::steady_clock::now();

        // Update state machine (check timeouts, etc.)
        m_stateMachine->update();

        // Update status
        updateStatus();

        // Publish status at configured rate
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusPublish);
        if (elapsed.count() >= statusIntervalMs) {
            publishStatus();
            lastStatusPublish = now;
        }

        // TODO: Motion control updates (Phase 2)

        // Calculate sleep time to maintain cycle rate
        auto cycleEnd = std::chrono::steady_clock::now();
        auto cycleDuration = std::chrono::duration_cast<std::chrono::milliseconds>(cycleEnd - cycleStart);
        int sleepMs = m_cycleTimeMs - static_cast<int>(cycleDuration.count());
        if (sleepMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }
    }

    LOG_DEBUG("Control loop stopped");
}

void RobotController::updateStatus() {
    std::lock_guard<std::mutex> lock(m_statusMutex);

    m_status.state = m_stateMachine->currentState();
    m_status.mode = m_stateMachine->currentMode();
    m_status.safety = m_stateMachine->safetyState();
    m_status.enabled = m_stateMachine->isEnabled();
    m_status.homed = m_stateMachine->isHomed();
    m_status.errorCode = m_stateMachine->lastError();
    m_status.errorMessage = m_stateMachine->lastErrorMessage();

    // TODO: Get actual joint positions from motion controller (Phase 2)
    // For now, use home position from config
    const auto& robotConfig = ConfigManager::instance().robotConfig();
    for (size_t i = 0; i < 6 && i < robotConfig.home_position.size(); i++) {
        m_status.jointPositions[i] = robotConfig.home_position[i];
    }
}

void RobotController::publishStatus() {
    if (!m_ipcServer) return;

    RobotStatus status = getStatus();

    nlohmann::json payload = {
        {"state", toString(status.state)},
        {"mode", toString(status.mode)},
        {"safety", toString(status.safety)},
        {"enabled", status.enabled},
        {"homed", status.homed},
        {"joints", status.jointPositions},
        {"tcp_position", status.tcpPose},
        {"speed_override", status.speedOverride},
        {"errors", nlohmann::json::array()}
    };

    if (status.errorCode != ErrorCode::NONE) {
        payload["errors"].push_back({
            {"code", static_cast<int>(status.errorCode)},
            {"message", status.errorMessage}
        });
    }

    m_ipcServer->publishStatus(payload);
}

void RobotController::registerIpcHandlers() {
    // GET_STATUS handler
    m_ipcServer->registerHandler(MessageType::GET_STATUS,
        [this](const Message& request) -> nlohmann::json {
            RobotStatus status = getStatus();
            return {
                {"state", toString(status.state)},
                {"mode", toString(status.mode)},
                {"safety", toString(status.safety)},
                {"enabled", status.enabled},
                {"homed", status.homed},
                {"joints", status.jointPositions},
                {"tcp_position", status.tcpPose},
                {"speed_override", status.speedOverride},
                {"error_code", static_cast<int>(status.errorCode)},
                {"error_message", status.errorMessage}
            };
        });

    // GET_JOINT_POSITIONS handler
    m_ipcServer->registerHandler(MessageType::GET_JOINT_POSITIONS,
        [this](const Message& request) -> nlohmann::json {
            RobotStatus status = getStatus();
            return {
                {"joints", status.jointPositions},
                {"unit", "degrees"}
            };
        });

    // GET_CONFIG handler
    m_ipcServer->registerHandler(MessageType::GET_CONFIG,
        [](const Message& request) -> nlohmann::json {
            return nlohmann::json::parse(ConfigManager::instance().robotConfigToJson());
        });

    // COMMAND handler
    m_ipcServer->registerHandler(MessageType::COMMAND,
        [this](const Message& request) -> nlohmann::json {
            std::string cmd = request.payload.value("command", "");
            bool success = false;
            std::string message;

            if (cmd == "enable") {
                success = enable();
                message = success ? "Robot enabled" : "Failed to enable";
            }
            else if (cmd == "disable") {
                success = disable();
                message = success ? "Robot disabled" : "Failed to disable";
            }
            else if (cmd == "reset") {
                success = reset();
                message = success ? "Reset complete" : "Failed to reset";
            }
            else if (cmd == "home") {
                success = home();
                message = success ? "Homing started" : "Failed to start homing";
            }
            else if (cmd == "set_mode") {
                std::string modeStr = request.payload.value("mode", "MANUAL");
                RobotMode mode = RobotMode::MANUAL;
                if (modeStr == "AUTO") mode = RobotMode::AUTO;
                else if (modeStr == "T1") mode = RobotMode::T1;
                else if (modeStr == "T2") mode = RobotMode::T2;
                success = setMode(mode);
                message = success ? "Mode changed" : "Failed to change mode";
            }
            else if (cmd == "set_speed") {
                double speed = request.payload.value("speed", 100.0);
                success = setSpeedOverride(speed);
                message = success ? "Speed set" : "Failed to set speed";
            }
            else {
                message = "Unknown command: " + cmd;
            }

            return {
                {"success", success},
                {"message", message}
            };
        });

    LOG_DEBUG("IPC handlers registered");
}

} // namespace robot_controller
