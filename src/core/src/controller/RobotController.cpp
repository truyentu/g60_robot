/**
 * @file RobotController.cpp
 * @brief Robot controller implementation
 */

#include "RobotController.hpp"
#include "../logging/Logger.hpp"
#include "../ipc/CatalogPayloads.hpp"
#include "../ipc/HomingPayloads.hpp"
#include "../ipc/ToolPayloads.hpp"
#include "../ipc/ModePayloads.hpp"
#include "../ipc/BasePayloads.hpp"
#include "../ipc/OverridePayloads.hpp"
#include "../ipc/RobotPackagePayloads.hpp"
#include "../config/RobotPackageLoader.hpp"
#include "../config/UrdfParser.hpp"
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>

namespace robot_controller {

using namespace state;
using namespace config;
using namespace ipc;

RobotController::RobotController()
    : m_stateMachine(std::make_unique<StateMachine>())
    , m_baseFrameManager(std::make_unique<frame::BaseFrameManager>())
    , m_overrideManager(std::make_unique<override::OverrideManager>())
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

    // Set robot library path for package loading
    std::filesystem::path robotLibPath = std::filesystem::path(configDir) / "robots";
    if (std::filesystem::exists(robotLibPath)) {
        RobotPackageLoader::setLibraryPath(robotLibPath);
    } else {
        // Try relative to executable
        robotLibPath = std::filesystem::current_path() / "config" / "robots";
        if (std::filesystem::exists(robotLibPath)) {
            RobotPackageLoader::setLibraryPath(robotLibPath);
        } else {
            LOG_WARN("Robot library path not found, package loading may fail");
        }
    }

    // Setup state machine callbacks
    m_stateMachine->setStateChangeCallback(
        [this](RobotState oldState, RobotState newState) {
            LOG_INFO("State changed: {} -> {}", toString(oldState), toString(newState));
            // Note: Don't call updateStatus() here to avoid deadlock
            // Status will be updated in control loop
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
    RobotStatus status = m_status;

    // Get override values from OverrideManager
    if (m_overrideManager) {
        status.programOverride = m_overrideManager->getProgramOverride();
        status.jogOverride = m_overrideManager->getJogOverride();
        status.manualOverride = m_overrideManager->getManualOverride();
    }

    return status;
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
    // Legacy: set all overrides to same value
    int value = static_cast<int>(std::clamp(percent, 1.0, 100.0));
    m_status.programOverride = value;
    m_status.jogOverride = value;
    m_status.manualOverride = value;
    LOG_DEBUG("Speed override set to {}%", value);
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

// ============================================================================
// Program Execution (Phase 8)
// ============================================================================

bool RobotController::loadProgram(const std::string& source) {
    // Create executor if needed
    if (!m_programExecutor) {
        m_programExecutor = std::make_unique<interpreter::Executor>();

        // Set up motion callback to integrate with trajectory generator
        m_programExecutor->setMotionCallback([this](const interpreter::MotionStmt& motion) {
            LOG_INFO("Motion command: {} to point", motion.type);
            // TODO: Send to trajectory generator in Phase 2
            // For now, just simulate motion time
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        });

        m_programExecutor->setLineCallback([this](int line) {
            LOG_DEBUG("Executing line {}", line);
        });

        m_programExecutor->setWaitCallback([](double seconds) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
        });

        m_programExecutor->setOutputCallback([this](int index, bool value) {
            LOG_INFO("$OUT[{}] = {}", index, value ? "TRUE" : "FALSE");
        });
    }

    // Tokenize
    interpreter::Lexer lexer(source);
    auto tokens = lexer.tokenize();

    if (lexer.hasErrors()) {
        for (const auto& err : lexer.getErrors()) {
            LOG_ERROR("Lexer error: {}", err);
        }
        return false;
    }

    // Parse
    interpreter::Parser parser(tokens);
    auto program = parser.parse();

    if (!program || parser.hasErrors()) {
        for (const auto& err : parser.getErrors()) {
            LOG_ERROR("Parser error: {}", err);
        }
        return false;
    }

    // Load into executor
    m_programExecutor->loadProgram(*program);
    LOG_INFO("Program loaded: {}", program->name);

    return true;
}

void RobotController::runProgram() {
    if (!m_programExecutor) {
        LOG_WARN("No program executor");
        return;
    }

    // Run in separate thread to not block control loop
    std::thread([this]() {
        m_programExecutor->run();
    }).detach();
}

void RobotController::stepProgram() {
    if (!m_programExecutor) {
        LOG_WARN("No program executor");
        return;
    }
    m_programExecutor->step();
}

void RobotController::pauseProgram() {
    if (m_programExecutor) {
        m_programExecutor->pause();
    }
}

void RobotController::stopProgram() {
    if (m_programExecutor) {
        m_programExecutor->stop();
    }
}

void RobotController::resetProgram() {
    if (m_programExecutor) {
        m_programExecutor->reset();
    }
}

interpreter::ExecutionState RobotController::getProgramState() const {
    if (m_programExecutor) {
        return m_programExecutor->getState();
    }
    return interpreter::ExecutionState::IDLE;
}

int RobotController::getProgramLine() const {
    if (m_programExecutor) {
        return m_programExecutor->getCurrentLine();
    }
    return 0;
}

std::string RobotController::getProgramName() const {
    if (m_programExecutor) {
        return m_programExecutor->getProgramName();
    }
    return "";
}

void RobotController::setPoint(const std::string& name, const std::vector<double>& values) {
    if (!m_programExecutor) {
        m_programExecutor = std::make_unique<interpreter::Executor>();
    }
    m_programExecutor->setPoint(name, values);
}

std::vector<double> RobotController::getPoint(const std::string& name) const {
    if (m_programExecutor) {
        return m_programExecutor->getPoint(name);
    }
    return {};
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

    // TODO: Calculate tcpInBase using KinematicsService + BaseFrameManager
    // Transform TCP from world frame to active base frame
    if (m_baseFrameManager) {
        // Create Frame from tcpPose
        frame::Frame worldTcp{
            m_status.tcpPose[0], m_status.tcpPose[1], m_status.tcpPose[2],
            m_status.tcpPose[3], m_status.tcpPose[4], m_status.tcpPose[5]
        };

        // Transform to active base frame
        frame::Frame baseTcp = m_baseFrameManager->transformToBase(worldTcp);

        m_status.tcpInBase[0] = baseTcp.x;
        m_status.tcpInBase[1] = baseTcp.y;
        m_status.tcpInBase[2] = baseTcp.z;
        m_status.tcpInBase[3] = baseTcp.rx;
        m_status.tcpInBase[4] = baseTcp.ry;
        m_status.tcpInBase[5] = baseTcp.rz;

        m_status.activeBaseId = m_baseFrameManager->getActiveBaseId();
    } else {
        // Fallback: tcpInBase = tcpPose (world frame = base frame)
        for (size_t i = 0; i < 6; i++) {
            m_status.tcpInBase[i] = m_status.tcpPose[i];
        }
    }

    // TODO: Get active tool from ToolManager
    // m_status.activeToolId is already initialized
}

void RobotController::publishStatus() {
    if (!m_ipcServer || !m_running) return;

    RobotStatus status = getStatus();

    nlohmann::json payload = {
        {"state", toString(status.state)},
        {"mode", toString(status.mode)},
        {"safety", toString(status.safety)},
        {"enabled", status.enabled},
        {"homed", status.homed},
        {"joints", status.jointPositions},
        {"tcp_position", status.tcpPose},
        {"tcp_in_base", status.tcpInBase},
        {"active_base_id", status.activeBaseId},
        {"active_tool_id", status.activeToolId},
        {"program_override", status.programOverride},
        {"jog_override", status.jogOverride},
        {"manual_override", status.manualOverride},
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
                {"tcp_in_base", status.tcpInBase},
                {"active_base_id", status.activeBaseId},
                {"active_tool_id", status.activeToolId},
                {"program_override", status.programOverride},
                {"jog_override", status.jogOverride},
                {"manual_override", status.manualOverride},
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

    // GET_ROBOT_CATALOG handler
    m_ipcServer->registerHandler(MessageType::GET_ROBOT_CATALOG,
        [](const Message& request) -> nlohmann::json {
            auto& config = ConfigManager::instance();
            const auto& catalog = config.robotCatalog();

            GetRobotCatalogResponse response;
            response.activeModelId = config.getActiveModelId();
            response.activeInstanceId = config.getActiveInstanceId();

            for (const auto& modelInfo : catalog.getAvailableModels()) {
                RobotModelSummary summary;
                summary.id = modelInfo.id;
                summary.name = modelInfo.name;
                summary.manufacturer = modelInfo.manufacturer;
                summary.dof = modelInfo.dof;
                summary.maxPayloadKg = modelInfo.maxPayloadKg;
                summary.reachMm = modelInfo.reachMm;
                response.models.push_back(summary);
            }

            return response;
        });

    // SELECT_ROBOT_MODEL handler
    m_ipcServer->registerHandler(MessageType::SELECT_ROBOT_MODEL,
        [this](const Message& request) -> nlohmann::json {
            SelectRobotModelRequest req = request.payload.get<SelectRobotModelRequest>();
            SelectRobotModelResponse response;

            auto& config = ConfigManager::instance();
            if (config.selectRobotModel(req.modelId)) {
                response.success = true;
                response.modelId = req.modelId;
                auto modelInfo = config.robotCatalog().getModelInfo(req.modelId);
                if (modelInfo) {
                    response.modelName = modelInfo->name;
                }

                // Publish config changed event
                RobotConfigChangedEvent event;
                event.modelId = req.modelId;
                event.modelName = response.modelName;
                event.instanceId = config.getActiveInstanceId();
                auto eventMsg = Message::create(MessageType::ROBOT_CONFIG_CHANGED, event);
                m_ipcServer->publish(eventMsg);
            } else {
                response.success = false;
                response.error = "Failed to select model: " + req.modelId;
            }

            return response;
        });

    // GET_ACTIVE_ROBOT handler
    m_ipcServer->registerHandler(MessageType::GET_ACTIVE_ROBOT,
        [](const Message& request) -> nlohmann::json {
            auto& config = ConfigManager::instance();
            GetActiveRobotResponse response;

            response.modelId = config.getActiveModelId();
            response.instanceId = config.getActiveInstanceId();

            auto modelInfo = config.robotCatalog().getModelInfo(response.modelId);
            if (modelInfo) {
                response.modelName = modelInfo->name;
                response.manufacturer = modelInfo->manufacturer;
                response.dof = modelInfo->dof;
                response.maxPayloadKg = modelInfo->maxPayloadKg;
                response.reachMm = modelInfo->reachMm;
            }

            return response;
        });

    // START_HOMING handler
    m_ipcServer->registerHandler(MessageType::START_HOMING,
        [this](const Message& request) -> nlohmann::json {
            StartHomingRequest req = request.payload.get<StartHomingRequest>();
            StartHomingResponse response;

            // TODO: Get HomingService instance and call startHoming
            // For now, return a placeholder response
            LOG_INFO("START_HOMING request: joint={}, method={}", req.jointIndex, req.method);

            // Simulate success for now
            response.success = true;
            response.error = "";

            return response;
        });

    // STOP_HOMING handler
    m_ipcServer->registerHandler(MessageType::STOP_HOMING,
        [this](const Message& request) -> nlohmann::json {
            StopHomingRequest req = request.payload.get<StopHomingRequest>();
            StopHomingResponse response;

            LOG_INFO("STOP_HOMING request: joint={}", req.jointIndex);

            response.success = true;
            response.error = "";

            return response;
        });

    // GET_HOMING_STATE handler
    m_ipcServer->registerHandler(MessageType::GET_HOMING_STATE,
        [this](const Message& request) -> nlohmann::json {
            HomingStateResponse response;

            // Return default state for now (will be connected to HomingService later)
            response.totalJoints = 6;
            response.homedCount = 0;
            response.allHomed = false;
            response.anyHoming = false;
            response.anyError = false;

            for (int i = 0; i < 6; ++i) {
                JointHomingStatusPayload joint;
                joint.jointIndex = i;
                joint.state = "NOT_HOMED";
                joint.progress = 0.0;
                joint.errorMessage = "";
                joint.limitSwitchActive = false;
                joint.currentPosition = 0.0;
                response.joints.push_back(joint);
            }

            return response;
        });

    // ========================================================================
    // Tool Management Handlers
    // ========================================================================

    // GET_TOOL_LIST handler
    m_ipcServer->registerHandler(MessageType::GET_TOOL_LIST,
        [this](const Message& request) -> nlohmann::json {
            GetToolListResponse response;

            // TODO: Get ToolManager instance and return tools
            // For now, return placeholder data
            ToolDataPayload defaultTool;
            defaultTool.id = "tool_default";
            defaultTool.name = "No Tool";
            defaultTool.description = "Default tool with no offset";
            defaultTool.isActive = true;

            response.tools.push_back(defaultTool);
            response.activeToolId = "tool_default";

            return response;
        });

    // GET_TOOL handler
    m_ipcServer->registerHandler(MessageType::GET_TOOL,
        [this](const Message& request) -> nlohmann::json {
            GetToolRequest req = request.payload.get<GetToolRequest>();
            GetToolResponse response;

            LOG_DEBUG("GET_TOOL request: toolId={}", req.toolId);

            if (req.toolId == "tool_default") {
                response.success = true;
                response.tool.id = "tool_default";
                response.tool.name = "No Tool";
                response.tool.isActive = true;
            } else {
                response.success = false;
                response.error = "Tool not found: " + req.toolId;
            }

            return response;
        });

    // CREATE_TOOL handler
    m_ipcServer->registerHandler(MessageType::CREATE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            CreateToolRequest req = request.payload.get<CreateToolRequest>();
            CreateToolResponse response;

            LOG_INFO("CREATE_TOOL request: id={}, name={}", req.tool.id, req.tool.name);

            // TODO: Connect to ToolManager
            response.success = true;

            return response;
        });

    // UPDATE_TOOL handler
    m_ipcServer->registerHandler(MessageType::UPDATE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            UpdateToolRequest req = request.payload.get<UpdateToolRequest>();
            UpdateToolResponse response;

            LOG_INFO("UPDATE_TOOL request: toolId={}", req.toolId);

            // TODO: Connect to ToolManager
            response.success = true;

            return response;
        });

    // DELETE_TOOL handler
    m_ipcServer->registerHandler(MessageType::DELETE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            DeleteToolRequest req = request.payload.get<DeleteToolRequest>();
            DeleteToolResponse response;

            LOG_INFO("DELETE_TOOL request: toolId={}", req.toolId);

            if (req.toolId == "tool_default") {
                response.success = false;
                response.error = "Cannot delete default tool";
            } else {
                // TODO: Connect to ToolManager
                response.success = true;
            }

            return response;
        });

    // SELECT_TOOL handler
    m_ipcServer->registerHandler(MessageType::SELECT_TOOL,
        [this](const Message& request) -> nlohmann::json {
            SelectToolRequest req = request.payload.get<SelectToolRequest>();
            SelectToolResponse response;

            LOG_INFO("SELECT_TOOL request: toolId={}", req.toolId);

            // TODO: Connect to ToolManager
            response.success = true;

            // Publish tool changed event
            ToolChangedEvent event;
            event.toolId = req.toolId;
            event.toolName = req.toolId;  // TODO: Get actual name
            auto eventMsg = Message::create(MessageType::TOOL_CHANGED, event);
            m_ipcServer->publish(eventMsg);

            return response;
        });

    // GET_ACTIVE_TOOL handler
    m_ipcServer->registerHandler(MessageType::GET_ACTIVE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            GetActiveToolResponse response;

            // TODO: Connect to ToolManager
            response.success = true;
            response.tool.id = "tool_default";
            response.tool.name = "No Tool";
            response.tool.isActive = true;

            return response;
        });

    // START_TCP_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::START_TCP_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            StartCalibrationRequest req = request.payload.get<StartCalibrationRequest>();
            StartCalibrationResponse response;

            LOG_INFO("START_TCP_CALIBRATION request: method={}", req.method);

            // TODO: Connect to ToolManager calibration
            response.success = true;
            response.pointsRequired = (req.method == "SIX_POINT") ? 6 : 4;

            return response;
        });

    // RECORD_CALIBRATION_POINT handler
    m_ipcServer->registerHandler(MessageType::RECORD_CALIBRATION_POINT,
        [this](const Message& request) -> nlohmann::json {
            RecordPointRequest req = request.payload.get<RecordPointRequest>();
            RecordPointResponse response;

            LOG_INFO("RECORD_CALIBRATION_POINT request");

            // TODO: Connect to ToolManager calibration
            response.success = true;
            response.pointIndex = 1;
            response.totalRequired = 4;
            response.isComplete = false;

            return response;
        });

    // FINISH_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::FINISH_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            FinishCalibrationResponse response;

            LOG_INFO("FINISH_CALIBRATION request");

            // TODO: Connect to ToolManager calibration
            response.success = true;
            response.calculatedTcp = ToolTCPPayload{0, 0, 100, 0, 0, 0};
            response.residualError = 0.5;

            return response;
        });

    // CANCEL_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::CANCEL_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("CANCEL_CALIBRATION request");

            // TODO: Connect to ToolManager
            return nlohmann::json{{"success", true}};
        });

    // GET_CALIBRATION_STATUS handler
    m_ipcServer->registerHandler(MessageType::GET_CALIBRATION_STATUS,
        [this](const Message& request) -> nlohmann::json {
            CalibrationStatusResponse response;

            // TODO: Connect to ToolManager
            response.state = "IDLE";
            response.method = "FOUR_POINT";
            response.pointsRecorded = 0;
            response.pointsRequired = 4;

            return response;
        });

    // ========================================================================
    // Operation Mode Handlers
    // ========================================================================

    // GET_OPERATION_MODE handler
    m_ipcServer->registerHandler(MessageType::GET_OPERATION_MODE,
        [this](const Message& request) -> nlohmann::json {
            GetOperationModeResponse response;

            // For now, return based on state machine mode
            // TODO: Connect to ModeManager
            auto status = getStatus();
            switch (status.mode) {
                case RobotMode::T1:
                    response.mode = "MANUAL";
                    response.maxLinearVelocity = 250.0;
                    response.maxJointVelocity = 30.0;
                    response.deadmanRequired = true;
                    break;
                case RobotMode::T2:
                    response.mode = "TEST";
                    response.maxLinearVelocity = 2000.0;
                    response.maxJointVelocity = 180.0;
                    response.deadmanRequired = false;
                    break;
                case RobotMode::AUTO:
                    response.mode = "AUTO";
                    response.maxLinearVelocity = 2000.0;
                    response.maxJointVelocity = 180.0;
                    response.deadmanRequired = false;
                    break;
                default:
                    response.mode = "MANUAL";
                    response.maxLinearVelocity = 250.0;
                    response.maxJointVelocity = 30.0;
                    response.deadmanRequired = true;
            }

            response.safetyFenceRequired = (response.mode == "AUTO" || response.mode == "REMOTE");
            response.externalControlAllowed = (response.mode == "REMOTE");

            return response;
        });

    // SET_OPERATION_MODE handler
    m_ipcServer->registerHandler(MessageType::SET_OPERATION_MODE,
        [this](const Message& request) -> nlohmann::json {
            SetOperationModeRequest req = request.payload.get<SetOperationModeRequest>();
            SetOperationModeResponse response;

            LOG_INFO("SET_OPERATION_MODE request: mode={}", req.mode);

            // Map mode string to RobotMode
            RobotMode targetMode = RobotMode::MANUAL;
            if (req.mode == "MANUAL") targetMode = RobotMode::T1;
            else if (req.mode == "TEST") targetMode = RobotMode::T2;
            else if (req.mode == "AUTO") targetMode = RobotMode::AUTO;
            else if (req.mode == "REMOTE") targetMode = RobotMode::AUTO;

            // Check preconditions
            auto status = getStatus();
            std::vector<std::string> missing;

            if (status.state == RobotState::MOVING || status.state == RobotState::EXECUTING) {
                missing.push_back("Robot must be stopped");
            }
            if (status.errorCode != ErrorCode::NONE) {
                missing.push_back("All alarms must be cleared");
            }
            if ((req.mode == "AUTO" || req.mode == "REMOTE") && !status.homed) {
                missing.push_back("Homing must be completed");
            }

            if (!missing.empty()) {
                response.success = false;
                response.error = "Cannot switch to " + req.mode;
                response.missingRequirements = missing;
                return response;
            }

            // Attempt mode change
            if (setMode(targetMode)) {
                response.success = true;
                response.newMode = req.mode;

                // Publish mode changed event
                OperationModeChangedEvent event;
                event.previousMode = toString(status.mode);
                event.newMode = req.mode;
                event.maxLinearVelocity = (req.mode == "MANUAL") ? 250.0 : 2000.0;
                event.maxJointVelocity = (req.mode == "MANUAL") ? 30.0 : 180.0;
                auto eventMsg = Message::create(MessageType::OPERATION_MODE_CHANGED, event);
                m_ipcServer->publish(eventMsg);
            } else {
                response.success = false;
                response.error = "Failed to change mode";
            }

            return response;
        });

    // GET_MODE_REQUIREMENTS handler
    m_ipcServer->registerHandler(MessageType::GET_MODE_REQUIREMENTS,
        [this](const Message& request) -> nlohmann::json {
            GetModeRequirementsRequest req = request.payload.get<GetModeRequirementsRequest>();
            GetModeRequirementsResponse response;

            response.targetMode = req.targetMode;

            auto status = getStatus();

            // Current system state
            response.robotStopped = (status.state != RobotState::MOVING &&
                                     status.state != RobotState::EXECUTING);
            response.safetyFenceClosed = true;  // TODO: Get from safety system
            response.deadmanReleased = true;    // TODO: Get from input
            response.noActiveAlarms = (status.errorCode == ErrorCode::NONE);
            response.homingComplete = status.homed;

            // Check missing requirements based on target mode
            if (!response.robotStopped) {
                response.missingItems.push_back("Robot must be stopped");
            }
            if (!response.noActiveAlarms) {
                response.missingItems.push_back("All alarms must be cleared");
            }

            if (req.targetMode == "AUTO" || req.targetMode == "REMOTE") {
                if (!response.safetyFenceClosed) {
                    response.missingItems.push_back("Safety fence must be closed");
                }
                if (!response.homingComplete) {
                    response.missingItems.push_back("Homing must be completed");
                }
            }

            if (req.targetMode == "TEST") {
                if (!response.homingComplete) {
                    response.missingItems.push_back("Homing must be completed");
                }
            }

            response.canTransition = response.missingItems.empty();

            return response;
        });

    // ========================================================================
    // Base Frame Handlers
    // ========================================================================

    // GET_BASE_LIST handler
    m_ipcServer->registerHandler(MessageType::GET_BASE_LIST,
        [this](const Message& request) -> nlohmann::json {
            GetBaseListResponse response;

            // TODO: Connect to BaseFrameManager
            // For now, return placeholder data
            BaseFramePayload world;
            world.id = "world";
            world.name = "World Frame";
            world.description = "Robot base coordinate system";
            world.isActive = true;

            response.bases.push_back(world);
            response.activeBaseId = "world";

            return response;
        });

    // GET_BASE handler
    m_ipcServer->registerHandler(MessageType::GET_BASE,
        [this](const Message& request) -> nlohmann::json {
            GetBaseRequest req = request.payload.get<GetBaseRequest>();
            GetBaseResponse response;

            LOG_DEBUG("GET_BASE request: baseId={}", req.baseId);

            if (req.baseId == "world") {
                response.success = true;
                response.base.id = "world";
                response.base.name = "World Frame";
                response.base.isActive = true;
            } else {
                response.success = false;
                response.error = "Base frame not found: " + req.baseId;
            }

            return response;
        });

    // CREATE_BASE handler
    m_ipcServer->registerHandler(MessageType::CREATE_BASE,
        [this](const Message& request) -> nlohmann::json {
            CreateBaseRequest req = request.payload.get<CreateBaseRequest>();
            CreateBaseResponse response;

            LOG_INFO("CREATE_BASE request: id={}, name={}", req.base.id, req.base.name);

            // TODO: Connect to BaseFrameManager
            response.success = true;

            return response;
        });

    // UPDATE_BASE handler
    m_ipcServer->registerHandler(MessageType::UPDATE_BASE,
        [this](const Message& request) -> nlohmann::json {
            UpdateBaseRequest req = request.payload.get<UpdateBaseRequest>();
            UpdateBaseResponse response;

            LOG_INFO("UPDATE_BASE request: baseId={}", req.baseId);

            // TODO: Connect to BaseFrameManager
            response.success = true;

            return response;
        });

    // DELETE_BASE handler
    m_ipcServer->registerHandler(MessageType::DELETE_BASE,
        [this](const Message& request) -> nlohmann::json {
            DeleteBaseRequest req = request.payload.get<DeleteBaseRequest>();
            DeleteBaseResponse response;

            LOG_INFO("DELETE_BASE request: baseId={}", req.baseId);

            if (req.baseId == "world") {
                response.success = false;
                response.error = "Cannot delete world frame";
            } else {
                // TODO: Connect to BaseFrameManager
                response.success = true;
            }

            return response;
        });

    // SELECT_BASE handler
    m_ipcServer->registerHandler(MessageType::SELECT_BASE,
        [this](const Message& request) -> nlohmann::json {
            SelectBaseRequest req = request.payload.get<SelectBaseRequest>();
            SelectBaseResponse response;

            LOG_INFO("SELECT_BASE request: baseId={}", req.baseId);

            // TODO: Connect to BaseFrameManager
            response.success = true;

            // Publish base changed event
            BaseChangedEvent event;
            event.baseId = req.baseId;
            event.baseName = req.baseId;
            auto eventMsg = Message::create(MessageType::BASE_CHANGED, event);
            m_ipcServer->publish(eventMsg);

            return response;
        });

    // GET_ACTIVE_BASE handler
    m_ipcServer->registerHandler(MessageType::GET_ACTIVE_BASE,
        [this](const Message& request) -> nlohmann::json {
            GetActiveBaseResponse response;

            // TODO: Connect to BaseFrameManager
            response.success = true;
            response.base.id = "world";
            response.base.name = "World Frame";
            response.base.isActive = true;

            return response;
        });

    // START_BASE_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::START_BASE_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            StartBaseCalibrationRequest req = request.payload.get<StartBaseCalibrationRequest>();
            StartBaseCalibrationResponse response;

            LOG_INFO("START_BASE_CALIBRATION request: method={}", req.method);

            // TODO: Connect to BaseFrameManager
            response.success = true;
            response.pointsRequired = (req.method == "FOUR_POINT") ? 4 : 3;

            return response;
        });

    // RECORD_BASE_POINT handler
    m_ipcServer->registerHandler(MessageType::RECORD_BASE_POINT,
        [this](const Message& request) -> nlohmann::json {
            RecordBasePointRequest req = request.payload.get<RecordBasePointRequest>();
            RecordBasePointResponse response;

            LOG_INFO("RECORD_BASE_POINT request: pointIndex={}", req.pointIndex);

            // TODO: Connect to BaseFrameManager
            response.success = true;
            response.pointIndex = req.pointIndex;
            response.pointName = (req.pointIndex == 0) ? "Origin" :
                                 (req.pointIndex == 1) ? "X-Direction" : "XY-Plane";
            response.totalPoints = 3;
            response.recordedPoints = req.pointIndex + 1;

            return response;
        });

    // FINISH_BASE_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::FINISH_BASE_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            FinishBaseCalibrationResponse response;

            LOG_INFO("FINISH_BASE_CALIBRATION request");

            // TODO: Connect to BaseFrameManager
            response.success = true;
            response.calculatedFrame = FramePayload{100, 50, 0, 0, 0, 45};

            return response;
        });

    // CANCEL_BASE_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::CANCEL_BASE_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("CANCEL_BASE_CALIBRATION request");

            // TODO: Connect to BaseFrameManager
            return nlohmann::json{{"success", true}};
        });

    // GET_BASE_CALIBRATION_STATUS handler
    m_ipcServer->registerHandler(MessageType::GET_BASE_CALIBRATION_STATUS,
        [this](const Message& request) -> nlohmann::json {
            BaseCalibrationStatusResponse response;

            // TODO: Connect to BaseFrameManager
            response.state = "IDLE";
            response.method = "THREE_POINT";
            response.pointsRequired = 3;
            response.pointsRecorded = 0;

            return response;
        });

    // ========================================================================
    // Override Control Handlers
    // ========================================================================

    // GET_OVERRIDE handler
    m_ipcServer->registerHandler(MessageType::GET_OVERRIDE,
        [this](const Message& request) -> nlohmann::json {
            GetOverrideResponse response;

            if (m_overrideManager) {
                response.programOverride = m_overrideManager->getProgramOverride();
                response.jogOverride = m_overrideManager->getJogOverride();
                response.manualOverride = m_overrideManager->getManualOverride();
            }

            return response;
        });

    // SET_OVERRIDE handler
    m_ipcServer->registerHandler(MessageType::SET_OVERRIDE,
        [this](const Message& request) -> nlohmann::json {
            SetOverrideRequest req = request.payload.get<SetOverrideRequest>();
            SetOverrideResponse response;

            LOG_INFO("SET_OVERRIDE request: program={}, jog={}, manual={}",
                     req.programOverride, req.jogOverride, req.manualOverride);

            if (m_overrideManager) {
                // Update only if value is valid (>= 1)
                if (req.programOverride >= 1) {
                    m_overrideManager->setProgramOverride(req.programOverride);
                }
                if (req.jogOverride >= 1) {
                    m_overrideManager->setJogOverride(req.jogOverride);
                }
                if (req.manualOverride >= 1) {
                    m_overrideManager->setManualOverride(req.manualOverride);
                }

                response.success = true;
                response.programOverride = m_overrideManager->getProgramOverride();
                response.jogOverride = m_overrideManager->getJogOverride();
                response.manualOverride = m_overrideManager->getManualOverride();
            } else {
                response.success = false;
                response.error = "OverrideManager not initialized";
            }

            // Publish override changed event
            OverrideChangedEvent event;
            event.programOverride = response.programOverride;
            event.jogOverride = response.jogOverride;
            event.manualOverride = response.manualOverride;
            event.changedBy = "user";
            auto eventMsg = Message::create(MessageType::OVERRIDE_CHANGED, event);
            m_ipcServer->publish(eventMsg);

            return response;
        });

    // ========================================================================
    // Robot Package Handlers (Virtual Simulation)
    // ========================================================================

    // GET_ROBOT_PACKAGES handler
    m_ipcServer->registerHandler(MessageType::GET_ROBOT_PACKAGES,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            auto packages = config::RobotPackageLoader::getBuiltInPackages();

            nlohmann::json packagesJson = nlohmann::json::array();
            for (const auto& pkg : packages) {
                packagesJson.push_back(packageInfoToJson(pkg));
            }

            response["success"] = true;
            response["packages"] = packagesJson;

            return response;
        });

    // LOAD_ROBOT_PACKAGE handler
    m_ipcServer->registerHandler(MessageType::LOAD_ROBOT_PACKAGE,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            std::string packageId = request.payload.value("package_id", "");

            LOG_INFO("LOAD_ROBOT_PACKAGE request: package_id={}", packageId);

            auto package = config::RobotPackageLoader::loadBuiltIn(packageId);
            if (!package) {
                response["success"] = false;
                response["error"] = "Package not found: " + packageId;
                return response;
            }

            // Store active package
            // TODO: Store in member variable for later use

            response["success"] = true;
            auto packageJson = packageToJson(*package);

            // DEBUG: Check if URDF data is in JSON
            LOG_DEBUG("Handler: packageJson has {} joints", packageJson["joints"].size());
            if (!packageJson["joints"].empty()) {
                bool hasUrdf = packageJson["joints"][0].contains("origin_xyz");
                LOG_DEBUG("Handler: First joint has origin_xyz: {}", hasUrdf);
            }

            response["package"] = packageJson;

            // Publish package changed event
            nlohmann::json eventPayload = {
                {"package_id", packageId},
                {"package_name", package->name}
            };
            auto eventMsg = Message::create(MessageType::ROBOT_PACKAGE_CHANGED, eventPayload);
            m_ipcServer->publish(eventMsg);

            return response;
        });

    // GET_ACTIVE_PACKAGE handler
    m_ipcServer->registerHandler(MessageType::GET_ACTIVE_PACKAGE,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            // TODO: Return stored active package
            // For now, return null
            response["success"] = true;
            response["package"] = nlohmann::json::object();

            return response;
        });

    // ========================================================================
    // Program Execution Handlers (Virtual Simulation)
    // ========================================================================

    // LOAD_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::LOAD_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            std::string source = request.payload.value("source", "");

            LOG_INFO("LOAD_PROGRAM request");

            if (source.empty()) {
                response["success"] = false;
                response["error"] = "No program source provided";
                return response;
            }

            if (loadProgram(source)) {
                response["success"] = true;
                response["program_name"] = getProgramName();
            } else {
                response["success"] = false;
                response["error"] = "Failed to parse program";
            }

            return response;
        });

    // RUN_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::RUN_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("RUN_PROGRAM request");

            runProgram();
            response["success"] = true;

            return response;
        });

    // STEP_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::STEP_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("STEP_PROGRAM request");

            stepProgram();
            response["success"] = true;
            response["state"] = interpreter::executionStateToString(getProgramState());
            response["current_line"] = getProgramLine();

            return response;
        });

    // PAUSE_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::PAUSE_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("PAUSE_PROGRAM request");

            pauseProgram();
            response["success"] = true;

            return response;
        });

    // STOP_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::STOP_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("STOP_PROGRAM request");

            stopProgram();
            response["success"] = true;

            return response;
        });

    // RESET_PROGRAM handler
    m_ipcServer->registerHandler(MessageType::RESET_PROGRAM,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("RESET_PROGRAM request");

            resetProgram();
            response["success"] = true;

            return response;
        });

    // GET_PROGRAM_STATE handler
    m_ipcServer->registerHandler(MessageType::GET_PROGRAM_STATE,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            response["state"] = interpreter::executionStateToString(getProgramState());
            response["current_line"] = getProgramLine();
            response["program_name"] = getProgramName();

            return response;
        });

    // SET_POINT handler
    m_ipcServer->registerHandler(MessageType::SET_POINT,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            std::string name = request.payload.value("name", "");
            std::vector<double> values = request.payload.value("values", std::vector<double>{});

            if (name.empty()) {
                response["success"] = false;
                response["error"] = "Point name is required";
                return response;
            }

            LOG_INFO("SET_POINT request: name={}", name);

            setPoint(name, values);
            response["success"] = true;

            return response;
        });

    // GET_POINTS handler
    m_ipcServer->registerHandler(MessageType::GET_POINTS,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            if (m_programExecutor) {
                const auto& points = m_programExecutor->getPoints();
                nlohmann::json pointsJson = nlohmann::json::object();
                for (const auto& [name, values] : points) {
                    pointsJson[name] = values;
                }
                response["points"] = pointsJson;
            } else {
                response["points"] = nlohmann::json::object();
            }

            return response;
        });

    // ========================================================================
    // URDF Import Handlers (Auto robot package creation)
    // ========================================================================

    // PARSE_URDF handler
    m_ipcServer->registerHandler(MessageType::PARSE_URDF,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            std::string urdfContent = request.payload.value("urdf_content", "");
            bool isFilePath = request.payload.value("is_file_path", false);

            LOG_INFO("PARSE_URDF request: is_file_path={}", isFilePath);

            config::UrdfParser parser;
            config::UrdfParseResult result;

            if (isFilePath) {
                result = parser.parseFile(urdfContent);
            } else {
                result = parser.parseString(urdfContent);
            }

            if (result.success) {
                response["success"] = true;
                response["error"] = "";

                // Convert model to JSON
                nlohmann::json modelJson;
                modelJson["name"] = result.model.name;
                modelJson["base_link"] = result.model.base_link_name;
                modelJson["joint_order"] = result.model.joint_order;

                nlohmann::json linksJson = nlohmann::json::array();
                for (const auto& link : result.model.links) {
                    linksJson.push_back({
                        {"name", link.name},
                        {"visual_mesh", link.visual_mesh},
                        {"collision_mesh", link.collision_mesh}
                    });
                }
                modelJson["links"] = linksJson;

                nlohmann::json jointsJson = nlohmann::json::array();
                for (const auto& joint : result.model.joints) {
                    jointsJson.push_back({
                        {"name", joint.name},
                        {"type", joint.type},
                        {"parent_link", joint.parent_link},
                        {"child_link", joint.child_link},
                        {"origin_xyz", {
                            config::UrdfParser::metersToMm(joint.origin_xyz[0]),
                            config::UrdfParser::metersToMm(joint.origin_xyz[1]),
                            config::UrdfParser::metersToMm(joint.origin_xyz[2])
                        }},
                        {"origin_rpy", {joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2]}},
                        {"axis", {joint.axis[0], joint.axis[1], joint.axis[2]}},
                        {"limit_lower_deg", config::UrdfParser::radiansToDegrees(joint.limit_lower)},
                        {"limit_upper_deg", config::UrdfParser::radiansToDegrees(joint.limit_upper)},
                        {"limit_velocity_deg", config::UrdfParser::radiansToDegrees(joint.limit_velocity)}
                    });
                }
                modelJson["joints"] = jointsJson;

                response["model"] = modelJson;
                LOG_INFO("URDF parsed successfully: {} links, {} joints",
                    result.model.links.size(), result.model.joints.size());
            } else {
                response["success"] = false;
                response["error"] = result.error;
                LOG_ERROR("URDF parse failed: {}", result.error);
            }

            return response;
        });

    // GENERATE_ROBOT_YAML handler
    m_ipcServer->registerHandler(MessageType::GENERATE_ROBOT_YAML,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            std::string urdfContent = request.payload.value("urdf_content", "");
            bool isFilePath = request.payload.value("is_file_path", false);
            std::string robotName = request.payload.value("robot_name", "");
            std::string manufacturer = request.payload.value("manufacturer", "Unknown");
            std::string outputPath = request.payload.value("output_path", "");

            LOG_INFO("GENERATE_ROBOT_YAML request: robot_name={}, output_path={}",
                robotName, outputPath);

            // Parse URDF first
            config::UrdfParser parser;
            config::UrdfParseResult result;

            if (isFilePath) {
                result = parser.parseFile(urdfContent);
            } else {
                result = parser.parseString(urdfContent);
            }

            if (!result.success) {
                response["success"] = false;
                response["error"] = "Failed to parse URDF: " + result.error;
                return response;
            }

            // Generate YAML
            std::string yamlContent = parser.generateYaml(result.model, robotName, manufacturer);

            response["success"] = true;
            response["yaml_content"] = yamlContent;

            // Save to file if path provided
            if (!outputPath.empty()) {
                try {
                    std::filesystem::path outPath(outputPath);

                    // Create directory if needed
                    auto parentDir = outPath.parent_path();
                    if (!parentDir.empty() && !std::filesystem::exists(parentDir)) {
                        std::filesystem::create_directories(parentDir);
                    }

                    std::ofstream outFile(outPath);
                    if (outFile.is_open()) {
                        outFile << yamlContent;
                        outFile.close();
                        response["saved_path"] = outputPath;
                        LOG_INFO("robot.yaml saved to: {}", outputPath);
                    } else {
                        response["saved_path"] = "";
                        LOG_WARN("Could not open file for writing: {}", outputPath);
                    }
                } catch (const std::exception& e) {
                    response["saved_path"] = "";
                    LOG_ERROR("Failed to save robot.yaml: {}", e.what());
                }
            }

            return response;
        });

    LOG_DEBUG("IPC handlers registered");
}

} // namespace robot_controller
