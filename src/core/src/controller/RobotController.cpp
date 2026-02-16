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
#include "../ipc/JogPayloads.hpp"
#include "../kinematics/UrdfForwardKinematics.hpp"
#include "../config/RobotPackageLoader.hpp"
#include "../config/UrdfParser.hpp"
#include "../kinematics/DHParameters.hpp"
// V2: STM32EthernetDriver for hardware connection
#include "../firmware/STM32EthernetDriver.hpp"
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace robot_controller {

using namespace state;
using namespace config;
using namespace ipc;

RobotController::RobotController()
    : m_stateMachine(std::make_unique<StateMachine>())
    , m_baseFrameManager(std::make_unique<frame::BaseFrameManager>())
    , m_overrideManager(std::make_unique<override::OverrideManager>())
    , m_simDriver(std::make_shared<firmware::FirmwareSimulator>())
    , m_jogController(std::make_unique<jog::JogController>())
{
    // Default to simulation mode
    m_activeDriver = m_simDriver;
    m_isSimMode = true;
    m_jogController->setFirmwareDriver(m_activeDriver);
    LOG_DEBUG("RobotController created (SIM mode)");
}

RobotController::~RobotController() {
    stop();
}

bool RobotController::initialize(const std::string& configDir) {
    LOG_INFO("Initializing RobotController...");

    // Determine effective paths — workspace takes priority
    std::string effectiveConfigDir = configDir;
    if (!m_workspacePath.empty()) {
        auto wsConfig = std::filesystem::path(m_workspacePath) / "Config" / "system.yaml";
        if (std::filesystem::exists(wsConfig)) {
            effectiveConfigDir = (std::filesystem::path(m_workspacePath) / "Config").string();
            LOG_INFO("Loading config from workspace: {}", effectiveConfigDir);
        } else {
            LOG_INFO("Workspace Config/system.yaml not found, falling back to: {}", configDir);
        }
    }

    // Load configuration
    auto& config = ConfigManager::instance();
    if (!config.loadAll(effectiveConfigDir)) {
        // Fallback to original configDir if workspace failed
        if (effectiveConfigDir != configDir) {
            LOG_WARN("Workspace config failed, falling back to: {}", configDir);
            if (!config.loadAll(configDir)) {
                LOG_ERROR("Failed to load configuration");
                return false;
            }
        } else {
            LOG_ERROR("Failed to load configuration");
            return false;
        }
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
    // Prefer workspace/Catalog/ if workspace is set, with configDir/robots as fallback
    bool robotLibFound = false;
    if (!m_workspacePath.empty()) {
        auto wsCatalog = std::filesystem::path(m_workspacePath) / "Catalog";
        if (std::filesystem::exists(wsCatalog)) {
            RobotPackageLoader::setLibraryPath(wsCatalog);
            LOG_INFO("Robot library from workspace: {}", wsCatalog.string());
            robotLibFound = true;

            // Set fallback to configDir/robots so packages not yet in Catalog can still load
            std::filesystem::path fallbackPath = std::filesystem::path(configDir) / "robots";
            if (!std::filesystem::exists(fallbackPath)) {
                fallbackPath = std::filesystem::current_path() / "config" / "robots";
            }
            if (std::filesystem::exists(fallbackPath)) {
                RobotPackageLoader::setFallbackLibraryPath(fallbackPath);
            }
        }
    }
    if (!robotLibFound) {
        std::filesystem::path robotLibPath = std::filesystem::path(configDir) / "robots";
        if (std::filesystem::exists(robotLibPath)) {
            RobotPackageLoader::setLibraryPath(robotLibPath);
        } else {
            robotLibPath = std::filesystem::current_path() / "config" / "robots";
            if (std::filesystem::exists(robotLibPath)) {
                RobotPackageLoader::setLibraryPath(robotLibPath);
            } else {
                LOG_WARN("Robot library path not found, package loading may fail");
            }
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

    // Initialize ToolManager
    m_toolManager = std::make_unique<tool::ToolManager>();
    m_toolManager->setToolChangedCallback([this](const std::string& toolId) {
        LOG_INFO("Tool changed to: {}", toolId);
        auto activeTool = m_toolManager->getActiveTool();
        if (activeTool && m_jogController) {
            if (activeTool->tcp.isZero()) {
                // No tool TCP: just flange offset
                m_jogController->setToolTransform(m_flangeTransform);
            } else {
                // T_composed = T_flangeOffset * T_toolTcp
                m_jogController->setToolTransform(m_flangeTransform * activeTool->tcp.toTransform());
            }
        }
        // Publish tool changed event via IPC
        if (m_ipcServer) {
            ipc::ToolChangedEvent event;
            event.toolId = toolId;
            event.toolName = activeTool ? activeTool->name : toolId;
            auto eventMsg = ipc::Message::create(ipc::MessageType::TOOL_CHANGED, event);
            m_ipcServer->publish(eventMsg);
        }
    });

    // Load tools — prefer workspace/Tools/ if available
    bool toolsLoaded = false;
    if (!m_workspacePath.empty()) {
        auto wsTools = std::filesystem::path(m_workspacePath) / "Tools";
        if (std::filesystem::exists(wsTools)) {
            m_toolManager->loadFromDirectory(wsTools.string());
            LOG_INFO("Tools loaded from workspace: {}", wsTools.string());
            toolsLoaded = true;
        }
    }
    if (!toolsLoaded) {
        std::filesystem::path toolsDir = std::filesystem::path(configDir) / "tools";
        if (std::filesystem::exists(toolsDir)) {
            m_toolManager->loadFromDirectory(toolsDir.string());
            LOG_INFO("Tools loaded from: {}", toolsDir.string());
        }
    }

    // Load base frames — prefer workspace/Frames/ if available
    bool framesLoaded = false;
    if (!m_workspacePath.empty()) {
        auto wsFrames = std::filesystem::path(m_workspacePath) / "Frames";
        if (std::filesystem::exists(wsFrames)) {
            m_baseFrameManager->loadFromDirectory(wsFrames.string());
            LOG_INFO("Frames loaded from workspace: {}", wsFrames.string());
            framesLoaded = true;
        }
    }
    if (!framesLoaded) {
        std::filesystem::path framesDir = std::filesystem::path(configDir) / "frames";
        if (std::filesystem::exists(framesDir)) {
            m_baseFrameManager->loadFromDirectory(framesDir.string());
            LOG_INFO("Frames loaded from: {}", framesDir.string());
        }
    }

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
        LOG_INFO("Go Home started (PTP move to park position)");

        std::thread([this]() {
            if (!m_activeDriver) {
                m_stateMachine->processEvent(StateEvent::HOME_COMPLETE);
                return;
            }

            // Determine home position ($H_POS equivalent)
            std::array<double, 6> homePos = {0, 0, 0, 0, 0, 0};
            if (m_activePackage && !m_activePackage->home_position.empty()) {
                for (size_t i = 0; i < 6 && i < m_activePackage->home_position.size(); i++) {
                    homePos[i] = m_activePackage->home_position[i];
                }
            }
            LOG_INFO("Go Home target ($H_POS): [{}, {}, {}, {}, {}, {}]",
                     homePos[0], homePos[1], homePos[2],
                     homePos[3], homePos[4], homePos[5]);

            // Get current joint positions
            auto currentPos = m_activeDriver->getJointPositions();

            // Interpolate from current to home position (smoothstep, 50Hz)
            const double homingSpeedDegPerSec = 60.0;  // Conservative homing speed
            double maxDelta = 0.0;
            for (int i = 0; i < 6; i++) {
                maxDelta = std::max(maxDelta, std::abs(homePos[i] - currentPos[i]));
            }

            double totalTime = (maxDelta > 0.1) ? (maxDelta / homingSpeedDegPerSec) : 0.1;
            totalTime = std::clamp(totalTime, 0.1, 10.0);

            const int hz = 50;
            int totalSteps = static_cast<int>(totalTime * hz);
            if (totalSteps < 1) totalSteps = 1;

            for (int step = 1; step <= totalSteps; step++) {
                double t = static_cast<double>(step) / totalSteps;
                double s = t * t * (3.0 - 2.0 * t);  // smoothstep

                std::array<double, 6> interpPos;
                for (int i = 0; i < 6; i++) {
                    interpPos[i] = currentPos[i] + s * (homePos[i] - currentPos[i]);
                }

                m_activeDriver->setJointPositionsDirect(interpPos);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / hz));
            }

            // Final: set exactly home position
            m_activeDriver->setJointPositionsDirect(homePos);

            m_stateMachine->processEvent(StateEvent::HOME_COMPLETE);
            LOG_INFO("Go Home complete - robot at park position");
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
    LOG_INFO("loadProgram: begin, source size={}", source.size());

    // Create executor if needed
    if (!m_programExecutor) {
        m_programExecutor = std::make_unique<interpreter::Executor>();

        // Set up motion callback - resolve point, IK, interpolate, update joints
        m_programExecutor->setMotionCallback([this](const interpreter::MotionStmt& motion) {
            // 1. Resolve target coordinates from expression
            std::vector<double> targetValues;
            std::string pointName = "(inline)";

            if (motion.target) {
                std::visit([&](auto&& e) {
                    using T = std::decay_t<decltype(e)>;
                    if constexpr (std::is_same_v<T, interpreter::PointExpr>) {
                        pointName = e.name;
                        if (!e.values.empty()) {
                            targetValues = e.values;
                        } else {
                            targetValues = m_programExecutor->getPoint(e.name);
                        }
                    }
                    else if constexpr (std::is_same_v<T, interpreter::VariableExpr>) {
                        pointName = e.name;
                        targetValues = m_programExecutor->getPoint(e.name);
                    }
                    else if constexpr (std::is_same_v<T, interpreter::AggregateExpr>) {
                        // Extract field values from aggregate {X 10, Y 20, Z 30, A 0, B 90, C 0}
                        for (const auto& [fieldName, fieldExpr] : e.fields) {
                            targetValues.push_back(m_programExecutor->evaluateExpression(fieldExpr));
                        }
                        pointName = e.typeName.empty() ? "(aggregate)" : e.typeName;
                    }
                }, *motion.target);
            }

            if (targetValues.size() < 3) {
                LOG_WARN("Motion {}: target '{}' not resolved or has < 3 values", motion.type, pointName);
                return;
            }

            // 2. Build target pose [x, y, z, rx, ry, rz] (mm, degrees)
            double tx = targetValues[0];
            double ty = targetValues[1];
            double tz = targetValues[2];

            // Extract orientation from target values
            // KRL E6POS format: {X, Y, Z, A, B, C} where A=rotZ, B=rotY, C=rotX (ZYX Euler, degrees)
            // ABB robtarget: {x, y, z, qw, qx, qy, qz, ...} (quaternion)
            double rx_deg = 0, ry_deg = 0, rz_deg = 0;
            if (targetValues.size() == 6) {
                // KRL E6POS: A=rotZ, B=rotY, C=rotX
                rz_deg = targetValues[3];  // A
                ry_deg = targetValues[4];  // B
                rx_deg = targetValues[5];  // C
            } else if (targetValues.size() >= 7) {
                // ABB quaternion convention
                double qw = targetValues[3];
                double qx = targetValues[4];
                double qy = targetValues[5];
                double qz = targetValues[6];

                double sinr_cosp = 2.0 * (qw * qx + qy * qz);
                double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
                double rx_rad = std::atan2(sinr_cosp, cosr_cosp);

                double sinp = 2.0 * (qw * qy - qz * qx);
                double ry_rad;
                if (std::abs(sinp) >= 1.0)
                    ry_rad = std::copysign(M_PI / 2, sinp);
                else
                    ry_rad = std::asin(sinp);

                double siny_cosp = 2.0 * (qw * qz + qx * qy);
                double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                double rz_rad = std::atan2(siny_cosp, cosy_cosp);

                rx_deg = rx_rad * 180.0 / M_PI;
                ry_deg = ry_rad * 180.0 / M_PI;
                rz_deg = rz_rad * 180.0 / M_PI;
            }

            std::array<double, 6> targetPose = {tx, ty, tz, rx_deg, ry_deg, rz_deg};

            LOG_INFO("Motion {} to '{}': [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]",
                     motion.type, pointName, tx, ty, tz, rx_deg, ry_deg, rz_deg);

            // Log CIRC-specific parameters
            if (motion.auxPoint) {
                LOG_INFO("  CIRC auxPoint present");
            }
            if (motion.circAngle) {
                double angle = m_programExecutor->evaluateExpression(motion.circAngle);
                LOG_INFO("  CIRC CA angle: {:.1f} deg", angle);
            }

            // 4. Check if kinematics available
            if (!m_jogController || !m_jogController->hasKinematics()) {
                LOG_WARN("Motion {}: kinematics not initialized, skipping", motion.type);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return;
            }

            // 5. Get current joint positions (degrees)
            auto currentJointsDeg = m_activeDriver->getJointPositions();
            std::array<double, 6> currentJoints;
            for (int i = 0; i < 6; i++) currentJoints[i] = currentJointsDeg[i];

            // 6. Compute IK for target
            // Try with current joints as seed first
            auto ikResult = m_jogController->computeIK(targetPose, currentJoints);

            // For LIN/CIRC: if first IK succeeded, verify the configuration is
            // geometrically sensible (J1 should point toward target XY).
            // If not, retry with J1 seeded from atan2(ty, tx).
            bool isLinearMotion = (motion.type != "PTP" && motion.type != "PTP_REL");
            if (isLinearMotion && ikResult.has_value()) {
                double j1Result = ikResult->angles[0] * 180.0 / M_PI;
                double j1Expected = std::atan2(ty, tx) * 180.0 / M_PI;
                double j1Diff = std::abs(j1Result - j1Expected);
                // Normalize to 0-180
                if (j1Diff > 180.0) j1Diff = 360.0 - j1Diff;

                if (j1Diff > 45.0) {
                    // J1 is far from expected direction - try with better seed
                    auto altSeed = currentJoints;
                    altSeed[0] = j1Expected;
                    auto altResult = m_jogController->computeIK(targetPose, altSeed);
                    if (altResult.has_value()) {
                        LOG_INFO("Motion {}: J1 re-seeded {:.1f} -> {:.1f} (expected {:.1f})",
                            motion.type, j1Result, altResult->angles[0] * 180.0 / M_PI, j1Expected);
                        ikResult = altResult;
                    }
                }
            }
            if (!ikResult.has_value()) {
                LOG_WARN("Motion {}: IK failed for point '{}' (unreachable)", motion.type, pointName);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return;
            }

            // 7. Convert IK result from radians to degrees
            std::array<double, 6> targetJointsDeg;
            for (int i = 0; i < 6; i++) {
                targetJointsDeg[i] = ikResult->angles[i] * 180.0 / M_PI;
            }

            LOG_INFO("Motion {} '{}': joints [{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}] -> [{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                motion.type, pointName,
                currentJointsDeg[0], currentJointsDeg[1], currentJointsDeg[2],
                currentJointsDeg[3], currentJointsDeg[4], currentJointsDeg[5],
                targetJointsDeg[0], targetJointsDeg[1], targetJointsDeg[2],
                targetJointsDeg[3], targetJointsDeg[4], targetJointsDeg[5]);

            // 8. Interpolate from current to target joints
            // Read velocity from KRL system variables
            double overrideFactor = m_programExecutor->getSystemVariable("OV_PRO") / 100.0;
            if (overrideFactor < 0.01) overrideFactor = 0.01;

            bool isJoint = (motion.type == "PTP" || motion.type == "PTP_REL");
            double speed_mm_s;
            if (isJoint) {
                // PTP: use $VEL_AXIS[1] (%), map to mm/s
                double velAxisPct = m_programExecutor->getSystemVariable("VEL_AXIS");
                speed_mm_s = (velAxisPct / 100.0) * 5000.0 * overrideFactor;  // scale: 100% = 5000 mm/s equiv
            } else {
                // LIN/CIRC: use $VEL.CP (m/s) -> mm/s
                double velCp = m_programExecutor->getSystemVariable("VEL", "CP");
                speed_mm_s = velCp * 1000.0 * overrideFactor;
            }

            // Calculate interpolation steps
            double stepTimeMs = 20.0;  // 50 Hz update rate
            double totalTimeMs;

            if (isJoint) {
                // Joint move: base on max joint delta
                double maxJointDelta = 0;
                for (int i = 0; i < 6; i++) {
                    maxJointDelta = std::max(maxJointDelta, std::abs(targetJointsDeg[i] - currentJointsDeg[i]));
                }
                double jointSpeed = std::min(speed_mm_s / 5000.0, 1.0) * 180.0;  // deg/s
                if (jointSpeed < 10.0) jointSpeed = 10.0;
                totalTimeMs = (maxJointDelta / jointSpeed) * 1000.0;
            } else {
                // Linear move: base on cartesian distance
                auto startTcp = m_jogController->getTcpPose();
                double dx = tx - startTcp[0];
                double dy = ty - startTcp[1];
                double dz = tz - startTcp[2];
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (speed_mm_s < 1.0) speed_mm_s = 100.0;
                totalTimeMs = (dist / speed_mm_s) * 1000.0;
            }

            // Clamp to reasonable range
            if (totalTimeMs < 100.0) totalTimeMs = 100.0;
            if (totalTimeMs > 10000.0) totalTimeMs = 10000.0;

            int numSteps = static_cast<int>(totalTimeMs / stepTimeMs);
            if (numSteps < 2) numSteps = 2;

            LOG_DEBUG("Motion {}: interpolating {} steps over {:.0f}ms",
                      motion.type, numSteps, totalTimeMs);

            // 9. Execute interpolation steps
            if (isJoint) {
                // PTP: joint-space interpolation (correct for joint moves)
                for (int step = 1; step <= numSteps; step++) {
                    if (m_programExecutor->getState() == interpreter::ExecutionState::STOPPED)
                        return;

                    double t = static_cast<double>(step) / numSteps;
                    double s = t * t * (3.0 - 2.0 * t);  // smoothstep

                    std::array<double, 6> interpJoints;
                    for (int i = 0; i < 6; i++) {
                        interpJoints[i] = currentJointsDeg[i] + s * (targetJointsDeg[i] - currentJointsDeg[i]);
                    }
                    m_activeDriver->setJointPositionsDirect(interpJoints);
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(stepTimeMs)));
                }
            } else {
                // LIN/CIRC: Cartesian-space interpolation with IK per step
                // This ensures TCP follows a straight line in Cartesian space
                auto linStartTcp = m_jogController->getTcpPose();
                double startX = linStartTcp[0], startY = linStartTcp[1], startZ = linStartTcp[2];
                double startRx = linStartTcp[3], startRy = linStartTcp[4], startRz = linStartTcp[5];

                // Use target IK joints (computed above) to guide seed interpolation.
                // This prevents the IK solver from getting stuck in wrong configuration
                // by giving it a seed that's a blend between current and target joints.
                auto prevJoints = currentJoints;

                LOG_INFO("LIN start TCP=[{:.1f},{:.1f},{:.1f}] ori=[{:.1f},{:.1f},{:.1f}] -> target TCP=[{:.1f},{:.1f},{:.1f}] ori=[{:.1f},{:.1f},{:.1f}]",
                    startX, startY, startZ, startRx, startRy, startRz,
                    tx, ty, tz, rx_deg, ry_deg, rz_deg);

                int ikFailCount = 0;
                for (int step = 1; step <= numSteps; step++) {
                    if (m_programExecutor->getState() == interpreter::ExecutionState::STOPPED)
                        return;

                    double t = static_cast<double>(step) / numSteps;
                    double s = t * t * (3.0 - 2.0 * t);  // smoothstep

                    // Interpolate Cartesian position and orientation
                    std::array<double, 6> interpPose;
                    interpPose[0] = startX + s * (tx - startX);
                    interpPose[1] = startY + s * (ty - startY);
                    interpPose[2] = startZ + s * (tz - startZ);
                    interpPose[3] = startRx + s * (rx_deg - startRx);
                    interpPose[4] = startRy + s * (ry_deg - startRy);
                    interpPose[5] = startRz + s * (rz_deg - startRz);

                    // Seed = blend of previous result and target joints
                    // This guides the solver toward the correct configuration
                    std::array<double, 6> blendSeed;
                    for (int i = 0; i < 6; i++) {
                        blendSeed[i] = prevJoints[i] + s * (targetJointsDeg[i] - prevJoints[i]);
                    }

                    auto stepIK = m_jogController->computeIK(interpPose, blendSeed);
                    if (stepIK.has_value()) {
                        std::array<double, 6> stepJointsDeg;
                        for (int i = 0; i < 6; i++) {
                            stepJointsDeg[i] = stepIK->angles[i] * 180.0 / M_PI;
                        }
                        m_activeDriver->setJointPositionsDirect(stepJointsDeg);
                        for (int i = 0; i < 6; i++) prevJoints[i] = stepJointsDeg[i];
                    } else {
                        ikFailCount++;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(stepTimeMs)));
                }
                if (ikFailCount > 0) {
                    LOG_WARN("LIN '{}': {} IK failures out of {} steps", pointName, ikFailCount, numSteps);
                }
            }

            LOG_DEBUG("Motion {} to '{}' completed", motion.type, pointName);

            // Update executor's current position for $POS_ACT and undo
            auto finalJoints = m_activeDriver->getJointPositions();
            std::vector<double> finalPos(finalJoints.begin(), finalJoints.end());
            m_programExecutor->updateCurrentPosition(finalPos);
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
    LOG_INFO("loadProgram: tokenizing...");
    interpreter::Lexer lexer(source);
    auto tokens = lexer.tokenize();
    LOG_INFO("loadProgram: tokenized, {} tokens, {} errors", tokens.size(), lexer.getErrors().size());

    if (lexer.hasErrors()) {
        for (const auto& err : lexer.getErrors()) {
            LOG_ERROR("Lexer error: {}", err);
        }
        return false;
    }

    // Parse
    LOG_INFO("loadProgram: parsing...");
    interpreter::Parser parser(tokens);
    auto program = parser.parse();
    LOG_INFO("loadProgram: parsed, hasProgram={}, errors={}", program.has_value(), parser.getErrors().size());

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

bool RobotController::switchToSimMode() {
    LOG_INFO("Switching to SIM mode...");

    // Stop any active jog motion (but keep jog mode enabled)
    if (m_jogController) {
        m_jogController->stopJog();
    }

    // Disconnect real driver if connected
    if (m_realDriver && m_realDriver->isConnected()) {
        // V2: generic disconnect — driver handles its own cleanup
        m_realDriver.reset();
    }

    // Switch to simulator
    m_activeDriver = m_simDriver;
    m_isSimMode = true;

    // Update jog controller
    if (m_jogController) {
        m_jogController->setFirmwareDriver(m_activeDriver);
    }

    LOG_INFO("Switched to SIM mode");
    return true;
}

bool RobotController::switchToRealMode(const std::string& portName) {
    // V2: switchToRealMode is now a placeholder
    // Use switchToSTM32Mode() for STM32 Ethernet connection (Step 8)
    LOG_WARN("switchToRealMode('{}') called — legacy Teensy driver removed. Use STM32 mode.", portName);
    return false;
}

std::string RobotController::getFirmwareMode() const {
    return m_isSimMode ? "SIM" : "STM32";
}

bool RobotController::switchToSTM32Mode(const std::string& ip, uint16_t port) {
    LOG_INFO("Switching to STM32 mode, ip={}:{}", ip, port);

    // Stop any active jog motion (but keep jog mode enabled)
    if (m_jogController) {
        m_jogController->stopJog();
    }

    // Create STM32 driver
    auto stm32Driver = std::make_shared<firmware::STM32EthernetDriver>();
    if (!stm32Driver->connect(ip, port)) {
        LOG_ERROR("Failed to connect to STM32 at {}:{}", ip, port);
        return false;
    }

    // Set packet log callback for UDP monitor
    // NOTE: Callback runs on IO thread — must NOT call m_ipcServer->publish() directly
    //       (ZMQ sockets are not thread-safe). Queue messages for control loop to publish.
    stm32Driver->setPacketLogCallback(
        [this](const std::string& dir, uint8_t type, uint8_t seq,
               const uint8_t* payload, uint16_t len) {
            std::string hexStr;
            uint16_t hexLen = std::min(len, static_cast<uint16_t>(32));
            hexStr.reserve(hexLen * 2);
            for (uint16_t i = 0; i < hexLen; i++) {
                char buf[3];
                snprintf(buf, sizeof(buf), "%02X", payload[i]);
                hexStr += buf;
            }
            if (len > 32) hexStr += "...";

            auto now = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();

            nlohmann::json log = {
                {"dir", dir},
                {"type", type},
                {"seq", seq},
                {"len", len},
                {"hex", hexStr},
                {"ts", ms}
            };

            // Thread-safe queue — control loop will drain and publish
            {
                std::lock_guard<std::mutex> lock(m_packetLogMutex);
                if (m_packetLogQueue.size() < 100) { // Cap queue size
                    m_packetLogQueue.push(ipc::Message::create(
                        ipc::MessageType::FIRMWARE_PACKET_LOG, log));
                }
            }
        });

    // Switch to STM32 driver
    m_realDriver = stm32Driver;
    m_activeDriver = m_realDriver;
    m_isSimMode = false;

    // Update jog controller
    if (m_jogController) {
        m_jogController->setFirmwareDriver(m_activeDriver);
    }

    LOG_INFO("Switched to STM32 mode at {}:{}", ip, port);
    return true;
}

bool RobotController::enableDrives(uint8_t axisMask) {
    if (!m_activeDriver) return false;
    LOG_INFO("Enabling drives (mask=0x{:02X})", axisMask);
    return m_activeDriver->enableDrives(axisMask);
}

bool RobotController::disableDrives(uint8_t axisMask) {
    if (!m_activeDriver) return false;
    LOG_INFO("Disabling drives (mask=0x{:02X})", axisMask);
    return m_activeDriver->disableDrives(axisMask);
}

bool RobotController::goHome(uint8_t axisMask) {
    if (!m_activeDriver) return false;
    LOG_INFO("Starting homing (mask=0x{:02X})", axisMask);
    return m_activeDriver->homeStart(axisMask, 0, 0);
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

        // Update jog controller and firmware simulator
        if (m_jogController) {
            m_jogController->update(m_cycleTimeMs / 1000.0);
        }

        // Drain packet log queue (thread-safe publish on control loop thread)
        {
            std::lock_guard<std::mutex> lock(m_packetLogMutex);
            while (!m_packetLogQueue.empty()) {
                m_ipcServer->publish(m_packetLogQueue.front());
                m_packetLogQueue.pop();
            }
        }

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

    // Get joint positions from firmware simulator if available
    if (m_jogController) {
        auto positions = m_jogController->getJointPositions();
        for (size_t i = 0; i < 6; i++) {
            m_status.jointPositions[i] = positions[i];
        }

        // Compute TCP pose via FK if kinematics available
        if (m_jogController->hasKinematics()) {
            auto tcpPose = m_jogController->getTcpPose();
            for (size_t i = 0; i < 6; i++) {
                m_status.tcpPose[i] = tcpPose[i];
            }
        }
    } else {
        // Fallback: use home position from config
        const auto& robotConfig = ConfigManager::instance().robotConfig();
        for (size_t i = 0; i < 6 && i < robotConfig.home_position.size(); i++) {
            m_status.jointPositions[i] = robotConfig.home_position[i];
        }
    }

    // TODO: Calculate tcpInBase using KinematicsService + BaseFrameManager
    // Transform TCP from world frame to active base frame
    if (m_baseFrameManager) {

        // Update active tool ID
        if (m_toolManager) {
            m_status.activeToolId = m_toolManager->getActiveToolId();
        }

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

    // V2: Update drive/home/IO status from firmware driver
    if (m_activeDriver) {
        auto statusPkt = m_activeDriver->getStatusPacket();
        m_status.driveReady = statusPkt.drive_ready;
        m_status.driveAlarm = statusPkt.drive_alarm;
        m_status.bufferLevel = statusPkt.pvt_buffer_lvl;
        m_status.digitalInputs = statusPkt.digital_inputs;
        m_status.digitalOutputs = statusPkt.digital_outputs;
        m_status.homeStatus = statusPkt.home_status;
        m_status.firmwareMode = m_isSimMode ? "SIM" : "STM32";
    }
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

    // Add jog state
    if (m_jogController) {
        auto jogState = m_jogController->getState();
        payload["jog_state"] = {
            {"enabled", jogState.enabled},
            {"is_moving", jogState.isMoving},
            {"current_mode", jogState.currentMode},
            {"current_axis", jogState.currentAxis},
            {"current_direction", jogState.currentDirection},
            {"current_speed", jogState.currentSpeed}
        };
    }

    // Add firmware mode info
    if (m_activeDriver) {
        payload["firmware"] = {
            {"mode", m_isSimMode ? "SIM" : "STM32"},
            {"connected", m_activeDriver->isConnected()},
            {"driver", m_activeDriver->getDriverName()},
            {"is_simulation", m_activeDriver->isSimulation()},
            {"drive_ready", status.driveReady},
            {"drive_alarm", status.driveAlarm},
            {"home_status", status.homeStatus},
            {"buffer_level", status.bufferLevel},
            {"digital_inputs", status.digitalInputs},
            {"digital_outputs", status.digitalOutputs}
        };
    }

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
            else if (cmd == "home" || cmd == "home_all" || cmd == "go_home") {
                // In SIM mode, auto-enable if not already enabled
                if (m_isSimMode && !m_stateMachine->isEnabled()) {
                    enable();
                    LOG_INFO("Auto-enabled robot for Go Home (SIM mode)");
                }
                success = home();
                message = success ? "Go Home started" : "Failed to start Go Home";
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

            auto tools = m_toolManager->getAllTools();
            for (const auto& tool : tools) {
                response.tools.push_back(ToolDataPayload::fromToolData(tool));
            }
            response.activeToolId = m_toolManager->getActiveToolId();

            return response;
        });

    // GET_TOOL handler
    m_ipcServer->registerHandler(MessageType::GET_TOOL,
        [this](const Message& request) -> nlohmann::json {
            GetToolRequest req = request.payload.get<GetToolRequest>();
            GetToolResponse response;

            LOG_DEBUG("GET_TOOL request: toolId={}", req.toolId);

            auto tool = m_toolManager->getTool(req.toolId);
            if (tool) {
                response.success = true;
                response.tool = ToolDataPayload::fromToolData(*tool);
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

            LOG_INFO("CREATE_TOOL request: id={}, name={}, visualMeshPath={}", req.tool.id, req.tool.name, req.tool.visualMeshPath);

            auto toolData = req.tool.toToolData();

            // Copy mesh file into config/tools/meshes/ if it's an absolute path
            if (!toolData.visualMeshPath.empty()) {
                namespace fs = std::filesystem;
                fs::path srcPath(toolData.visualMeshPath);
                if (srcPath.is_absolute() && fs::exists(srcPath)) {
                    auto& config = ConfigManager::instance();
                    fs::path toolsDir = fs::path(config.configDir()) / "tools" / "meshes";
                    fs::create_directories(toolsDir);

                    // Use tool ID + original extension as filename
                    std::string destFilename = toolData.id + srcPath.extension().string();
                    fs::path destPath = toolsDir / destFilename;

                    try {
                        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
                        // Store relative path: meshes/<filename>
                        toolData.visualMeshPath = "meshes/" + destFilename;
                        LOG_INFO("[CREATE_TOOL] Copied mesh to: {}, relative path: {}",
                                 destPath.string(), toolData.visualMeshPath);
                    } catch (const std::exception& e) {
                        LOG_WARN("[CREATE_TOOL] Failed to copy mesh: {}, keeping original path", e.what());
                    }
                }
            }

            response.success = m_toolManager->createTool(toolData);
            if (!response.success) {
                response.error = "Failed to create tool: " + req.tool.id;
            }

            return response;
        });

    // UPDATE_TOOL handler
    m_ipcServer->registerHandler(MessageType::UPDATE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            UpdateToolRequest req = request.payload.get<UpdateToolRequest>();
            UpdateToolResponse response;

            LOG_INFO("UPDATE_TOOL request: toolId={}", req.toolId);

            auto toolData = req.tool.toToolData();
            response.success = m_toolManager->updateTool(req.toolId, toolData);
            if (!response.success) {
                response.error = "Failed to update tool: " + req.toolId;
            }

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
                response.success = m_toolManager->deleteTool(req.toolId);
                if (!response.success) {
                    response.error = "Failed to delete tool: " + req.toolId;
                }
            }

            return response;
        });

    // SELECT_TOOL handler
    m_ipcServer->registerHandler(MessageType::SELECT_TOOL,
        [this](const Message& request) -> nlohmann::json {
            SelectToolRequest req = request.payload.get<SelectToolRequest>();
            SelectToolResponse response;

            LOG_INFO("SELECT_TOOL request: toolId={}", req.toolId);

            response.success = m_toolManager->setActiveTool(req.toolId);
            if (!response.success) {
                response.error = "Failed to select tool: " + req.toolId;
            }
            // Note: ToolChangedCallback (set in initialize()) handles:
            // - JogController::setToolTransform()
            // - Publishing TOOL_CHANGED IPC event

            return response;
        });

    // GET_ACTIVE_TOOL handler
    m_ipcServer->registerHandler(MessageType::GET_ACTIVE_TOOL,
        [this](const Message& request) -> nlohmann::json {
            GetActiveToolResponse response;

            auto activeTool = m_toolManager->getActiveTool();
            if (activeTool) {
                response.success = true;
                response.tool = ToolDataPayload::fromToolData(*activeTool);
                LOG_INFO("GET_ACTIVE_TOOL: id={}, name={}, visualMeshPath='{}'",
                    response.tool.id, response.tool.name, response.tool.visualMeshPath);
            } else {
                response.success = false;
                response.error = "No active tool";
            }

            return response;
        });

    // START_TCP_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::START_TCP_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            StartCalibrationRequest req = request.payload.get<StartCalibrationRequest>();
            StartCalibrationResponse response;

            LOG_INFO("START_TCP_CALIBRATION request: method={}", req.method);

            tool::CalibrationMethod method = tool::CalibrationMethod::FOUR_POINT;
            if (req.method == "SIX_POINT") {
                method = tool::CalibrationMethod::SIX_POINT;
            }

            response.success = m_toolManager->startCalibration(method);
            response.pointsRequired = (method == tool::CalibrationMethod::SIX_POINT) ? 6 : 4;

            return response;
        });

    // RECORD_CALIBRATION_POINT handler
    m_ipcServer->registerHandler(MessageType::RECORD_CALIBRATION_POINT,
        [this](const Message& request) -> nlohmann::json {
            RecordPointRequest req = request.payload.get<RecordPointRequest>();
            RecordPointResponse response;

            LOG_INFO("RECORD_CALIBRATION_POINT request");

            // Get current joint positions from jog controller
            auto joints = m_jogController->getJointPositions();
            response.success = m_toolManager->recordCalibrationPoint(joints);

            auto status = m_toolManager->getCalibrationStatus();
            response.pointIndex = status.pointsRecorded;
            response.totalRequired = status.pointsRequired;
            response.isComplete = (status.pointsRecorded >= status.pointsRequired);

            return response;
        });

    // FINISH_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::FINISH_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            FinishCalibrationResponse response;

            LOG_INFO("FINISH_CALIBRATION request");

            auto tcpResult = m_toolManager->calculateTCP();
            if (tcpResult) {
                response.success = true;
                response.calculatedTcp = ToolTCPPayload{
                    tcpResult->x, tcpResult->y, tcpResult->z,
                    tcpResult->rx, tcpResult->ry, tcpResult->rz
                };
                // Calculate residual error using calibration data
                response.residualError = 0.0;  // TODO: get from calibration
            } else {
                response.success = false;
                response.error = "Calibration calculation failed";
            }

            return response;
        });

    // CANCEL_CALIBRATION handler
    m_ipcServer->registerHandler(MessageType::CANCEL_CALIBRATION,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("CANCEL_CALIBRATION request");

            m_toolManager->cancelCalibration();
            return nlohmann::json{{"success", true}};
        });

    // GET_CALIBRATION_STATUS handler
    m_ipcServer->registerHandler(MessageType::GET_CALIBRATION_STATUS,
        [this](const Message& request) -> nlohmann::json {
            CalibrationStatusResponse response;

            auto status = m_toolManager->getCalibrationStatus();
            switch (status.state) {
                case tool::CalibrationState::IDLE: response.state = "IDLE"; break;
                case tool::CalibrationState::COLLECTING_POINTS: response.state = "COLLECTING"; break;
                case tool::CalibrationState::CALCULATING: response.state = "CALCULATING"; break;
                case tool::CalibrationState::COMPLETED: response.state = "COMPLETE"; break;
                case tool::CalibrationState::FAILED: response.state = "ERROR"; break;
            }
            switch (status.method) {
                case tool::CalibrationMethod::FOUR_POINT: response.method = "FOUR_POINT"; break;
                case tool::CalibrationMethod::SIX_POINT: response.method = "SIX_POINT"; break;
                default: response.method = "DIRECT_INPUT"; break;
            }
            response.pointsRecorded = status.pointsRecorded;
            response.pointsRequired = status.pointsRequired;

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

    // RELOAD_PACKAGES handler - Re-scan packages folder without restart
    m_ipcServer->registerHandler(MessageType::RELOAD_PACKAGES,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("RELOAD_PACKAGES request: Re-scanning packages folder...");

            // Re-initialize the package loader to scan for new packages
            config::RobotPackageLoader::reload();

            auto packages = config::RobotPackageLoader::getBuiltInPackages();

            nlohmann::json packagesJson = nlohmann::json::array();
            for (const auto& pkg : packages) {
                packagesJson.push_back(packageInfoToJson(pkg));
            }

            response["success"] = true;
            response["packages"] = packagesJson;
            response["count"] = packages.size();

            LOG_INFO("RELOAD_PACKAGES complete: {} packages found", packages.size());

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

            // Initialize kinematics from package DH parameters
            if (m_jogController && !package->joints.empty()) {
                kinematics::RobotKinematicConfig kinConfig;

                for (size_t i = 0; i < package->joints.size() && i < 6; i++) {
                    const auto& joint = package->joints[i];
                    double alphaRad = joint.dh_alpha * kinematics::DEG_TO_RAD;
                    double thetaOffsetRad = joint.dh_theta_offset * kinematics::DEG_TO_RAD;
                    double minRad = joint.limit_min * kinematics::DEG_TO_RAD;
                    double maxRad = joint.limit_max * kinematics::DEG_TO_RAD;

                    kinConfig.dhParams.emplace_back(
                        joint.dh_a,         // a (mm)
                        alphaRad,            // alpha (rad)
                        joint.dh_d,          // d (mm)
                        thetaOffsetRad,      // theta offset (rad)
                        minRad,              // min angle (rad)
                        maxRad,              // max angle (rad)
                        joint.name,          // name
                        (joint.type == "revolute") // isRevolute
                    );

                    // Set joint limits and velocities on JogController
                    m_jogController->setJointLimits(static_cast<int>(i), joint.limit_min, joint.limit_max);
                    m_jogController->setMaxVelocity(static_cast<int>(i), joint.velocity_max);
                }

                // Set flange/tool offset
                kinConfig.toolOffset = kinematics::Vector3d(
                    package->flange_offset[0],
                    package->flange_offset[1],
                    package->flange_offset[2]
                );

                m_jogController->initializeKinematics(kinConfig);
                LOG_INFO("Kinematics initialized for package '{}' with {} joints",
                         package->name, kinConfig.dhParams.size());

                // Initialize URDF FK (primary FK, replaces DH FK)
                // toolOffset = Zero: flange_offset is composed into m_toolTransform instead,
                // to avoid double-counting (FK already includes flange_offset + tool also includes it)
                auto urdfJoints = kinematics::buildUrdfJointsFromPackage(*package);
                if (!urdfJoints.empty()) {
                    kinematics::Vector3d zeroOffset = kinematics::Vector3d::Zero();
                    m_jogController->initializeUrdfKinematics(urdfJoints, zeroOffset);
                    LOG_INFO("URDF kinematics initialized for package '{}' with {} joints (flange_offset in toolTransform)",
                             package->name, urdfJoints.size());
                } else {
                    LOG_WARN("No URDF origin data found in package '{}', using DH kinematics as fallback",
                             package->name);
                }
            }

            // Store active package for homing and other operations
            m_activePackage = *package;

            // Store flange transform for composing with tool TCP
            m_flangeTransform = Eigen::Matrix4d::Identity();
            m_flangeTransform(0,3) = package->flange_offset[0];
            m_flangeTransform(1,3) = package->flange_offset[1];
            m_flangeTransform(2,3) = package->flange_offset[2];
            LOG_INFO("Flange offset stored: ({}, {}, {})",
                     package->flange_offset[0], package->flange_offset[1], package->flange_offset[2]);

            // Apply flange transform to JogController (with or without active tool)
            if (m_toolManager) {
                auto activeTool = m_toolManager->getActiveTool();
                if (activeTool && !activeTool->tcp.isZero()) {
                    // T_composed = T_flangeOffset * T_toolTcp
                    m_jogController->setToolTransform(m_flangeTransform * activeTool->tcp.toTransform());
                } else {
                    // No tool: just flange offset
                    m_jogController->setToolTransform(m_flangeTransform);
                }
            } else {
                m_jogController->setToolTransform(m_flangeTransform);
            }

            // Save last active package ID for auto-load on next startup
            {
                auto& cfg = ConfigManager::instance();
                if (!cfg.configDir().empty()) {
                    std::filesystem::path lastPkgFile = std::filesystem::path(cfg.configDir()) / "last_active_package.txt";
                    std::ofstream f(lastPkgFile);
                    if (f.is_open()) {
                        f << packageId;
                        LOG_INFO("Saved last active package: {}", packageId);
                    }
                }
            }

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

            if (m_activePackage) {
                response["success"] = true;
                response["package"] = packageToJson(*m_activePackage);
                response["package_id"] = m_activePackage->id;
            } else {
                response["success"] = true;
                response["package"] = nlohmann::json::object();
                response["package_id"] = "";
            }

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

            LOG_INFO("LOAD_PROGRAM request, source length={}", source.size());

            if (source.empty()) {
                response["success"] = false;
                response["error"] = "No program source provided";
                LOG_WARN("LOAD_PROGRAM: empty source");
                return response;
            }

            LOG_INFO("LOAD_PROGRAM: calling loadProgram...");
            if (loadProgram(source)) {
                response["success"] = true;
                response["program_name"] = getProgramName();
                LOG_INFO("LOAD_PROGRAM: success, name={}", getProgramName());
            } else {
                response["success"] = false;
                response["error"] = "Failed to parse program";
                LOG_WARN("LOAD_PROGRAM: parse failed");
            }

            LOG_INFO("LOAD_PROGRAM: returning response");
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

            // Include variables/points if requested (avoid overhead on polling)
            bool includeVars = false;
            if (request.payload.contains("include_variables")) {
                includeVars = request.payload["include_variables"].get<bool>();
            } else if (request.payload.contains("includeVariables")) {
                includeVars = request.payload["includeVariables"].get<bool>();
            }

            if (includeVars && m_programExecutor) {
                // User variables
                nlohmann::json varsJson = nlohmann::json::object();
                for (const auto& [name, value] : m_programExecutor->getVariables()) {
                    varsJson[name] = value;
                }
                response["variables"] = varsJson;

                // System variables
                const auto& sysVars = m_programExecutor->getSystemVars();
                nlohmann::json sysJson = nlohmann::json::object();
                sysJson["$OV_PRO"] = sysVars.ovPro;
                sysJson["$VEL.CP"] = sysVars.velCp;
                sysJson["$ACC.CP"] = sysVars.accCp;
                sysJson["$APO.CDIS"] = sysVars.apoCdis;
                sysJson["$APO.CPTP"] = sysVars.apoCptp;
                sysJson["$TOOL"] = sysVars.toolIndex;
                sysJson["$BASE"] = sysVars.baseIndex;
                response["system_variables"] = sysJson;

                // Points
                nlohmann::json pointsJson = nlohmann::json::object();
                for (const auto& [name, values] : m_programExecutor->getPoints()) {
                    pointsJson[name] = values;
                }
                response["points"] = pointsJson;
            }

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

    // BLOCK_SELECT handler (Satzanwahl - jump to line)
    m_ipcServer->registerHandler(MessageType::BLOCK_SELECT,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            int line = request.payload.value("line", 0);

            LOG_INFO("BLOCK_SELECT request: line={}", line);

            if (!m_programExecutor) {
                response["success"] = false;
                response["error"] = "No program loaded";
                return response;
            }

            if (line <= 0) {
                response["success"] = false;
                response["error"] = "Invalid line number";
                return response;
            }

            m_programExecutor->blockSelect(line);
            response["success"] = true;
            response["state"] = interpreter::executionStateToString(m_programExecutor->getState());
            response["current_line"] = m_programExecutor->getCurrentLine();

            return response;
        });

    // BACKWARD_STEP handler (BWD - undo last step)
    m_ipcServer->registerHandler(MessageType::BACKWARD_STEP,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            LOG_INFO("BACKWARD_STEP request");

            if (!m_programExecutor) {
                response["success"] = false;
                response["error"] = "No program loaded";
                return response;
            }

            if (!m_programExecutor->canBackward()) {
                response["success"] = false;
                response["error"] = "No steps to undo";
                return response;
            }

            m_programExecutor->backward();
            response["success"] = true;
            response["state"] = interpreter::executionStateToString(m_programExecutor->getState());
            response["current_line"] = m_programExecutor->getCurrentLine();

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

    // ========================================================================
    // Jog Control Handlers
    // ========================================================================

    // JOG_START handler - Enable jog mode
    m_ipcServer->registerHandler(MessageType::JOG_START,
        [this](const Message& request) -> nlohmann::json {
            auto req = request.payload.get<ipc::JogStartRequest>();

            LOG_INFO("JOG_START request: enableDeadman={}", req.enableDeadman);

            // Debug file logging
            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_START: enableDeadman=" << req.enableDeadman
                    << " firmwareDriver=" << (m_jogController ? "exists" : "null")
                    << " stateMachine.state=" << toString(m_stateMachine->currentState())
                    << " stateMachine.enabled=" << m_stateMachine->isEnabled()
                    << " stateMachine.homed=" << m_stateMachine->isHomed()
                    << " stateMachine.canJog=" << m_stateMachine->canJog()
                    << std::endl;
            }

            bool success = m_jogController->enable();

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_START result: success=" << success << std::endl;
            }

            ipc::JogStartResponse response{success, success ? "" : "Failed to enable jog mode"};
            return response;
        });

    // JOG_STOP handler - Stop jog motion (keep jog mode enabled)
    m_ipcServer->registerHandler(MessageType::JOG_STOP,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("JOG_STOP request");

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_STOP received" << std::endl;
            }

            m_jogController->stopJog();
            // Don't disable jog mode - just stop current motion

            ipc::JogStartResponse response{true, ""};
            return response;
        });

    // JOG_MOVE handler - Continuous jog
    m_ipcServer->registerHandler(MessageType::JOG_MOVE,
        [this](const Message& request) -> nlohmann::json {
            auto req = request.payload.get<ipc::JogMoveRequest>();

            LOG_DEBUG("JOG_MOVE request: mode={} axis={} dir={} speed={}% frame={}",
                      req.mode, req.axis, req.direction, req.speedPercent, req.frame);

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_MOVE: mode=" << req.mode << " axis=" << req.axis
                    << " dir=" << req.direction << " speed=" << req.speedPercent
                    << " jogEnabled=" << m_jogController->getState().enabled
                    << std::endl;
            }

            std::string error = m_jogController->startContinuousJog(
                req.mode, req.axis, req.direction, req.speedPercent, req.frame);

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_MOVE result: error='" << error << "'" << std::endl;
            }

            ipc::JogMoveResponse response{error.empty(), error};
            return response;
        });

    // JOG_STEP handler - Incremental jog
    m_ipcServer->registerHandler(MessageType::JOG_STEP,
        [this](const Message& request) -> nlohmann::json {
            auto req = request.payload.get<ipc::JogStepRequest>();

            LOG_DEBUG("JOG_STEP request: mode={} axis={} dir={} inc={} speed={}% frame={}",
                      req.mode, req.axis, req.direction, req.increment, req.speedPercent, req.frame);

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_STEP: mode=" << req.mode << " axis=" << req.axis
                    << " dir=" << req.direction << " inc=" << req.increment
                    << " speed=" << req.speedPercent
                    << " jogEnabled=" << m_jogController->getState().enabled
                    << std::endl;
            }

            std::string error = m_jogController->jogStep(
                req.mode, req.axis, req.direction,
                req.increment, req.speedPercent, req.frame);

            {
                std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
                dbg << "[CORE] JOG_STEP result: error='" << error << "'" << std::endl;
            }

            ipc::JogMoveResponse response{error.empty(), error};
            return response;
        });

    // ========================================================================
    // Kinematics Handlers (3D Jogging)
    // ========================================================================

    // COMPUTE_IK handler - Compute inverse kinematics for target pose
    m_ipcServer->registerHandler(MessageType::COMPUTE_IK,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            // Parse target pose [x,y,z,rx,ry,rz] in mm/degrees
            auto targetPose = request.payload.value("target_pose", std::array<double, 6>{0});
            auto currentJoints = request.payload.value("current_joints", std::array<double, 6>{0});

            LOG_DEBUG("COMPUTE_IK request: target=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                      targetPose[0], targetPose[1], targetPose[2],
                      targetPose[3], targetPose[4], targetPose[5]);

            if (!m_jogController || !m_jogController->hasKinematics()) {
                response["success"] = false;
                response["error"] = "Kinematics not initialized";
                response["joints"] = std::array<double, 6>{0};
                return response;
            }

            auto ikResult = m_jogController->computeIK(targetPose, currentJoints);
            if (ikResult.has_value()) {
                // Convert result from radians to degrees
                std::array<double, 6> jointsDeg;
                for (int i = 0; i < 6; i++) {
                    jointsDeg[i] = ikResult->angles[i] * 180.0 / M_PI;
                }

                // Apply joints to firmware driver if requested
                bool apply = request.payload.value("apply", false);
                if (apply && m_activeDriver) {
                    m_activeDriver->setJointPositionsDirect(jointsDeg);
                }

                response["success"] = true;
                response["joints"] = jointsDeg;
                response["residual_error"] = ikResult->residualError;
                response["iterations"] = ikResult->iterations;
                response["error"] = "";
            } else {
                response["success"] = false;
                response["joints"] = std::array<double, 6>{0};
                response["error"] = "IK solution not found (target unreachable)";
            }

            return response;
        });

    // SET_JOINTS handler - Directly set joint positions on firmware driver
    m_ipcServer->registerHandler(MessageType::SET_JOINTS,
        [this](const Message& request) -> nlohmann::json {
            nlohmann::json response;

            auto joints = request.payload.value("joints", std::array<double, 6>{0});

            LOG_DEBUG("SET_JOINTS request: [{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                      joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

            if (!m_activeDriver) {
                response["success"] = false;
                response["error"] = "No active firmware driver";
                return response;
            }

            m_activeDriver->setJointPositionsDirect(joints);
            response["success"] = true;
            response["error"] = "";
            return response;
        });

    // ========================================================================
    // Firmware Control Handlers
    // ========================================================================

    // FIRMWARE_CONNECT handler
    m_ipcServer->registerHandler(MessageType::FIRMWARE_CONNECT,
        [this](const Message& request) -> nlohmann::json {
            std::string port = request.payload.value("port", "");
            int baudRate = request.payload.value("baud_rate", 115200);

            LOG_INFO("FIRMWARE_CONNECT request: port={}, baud={}", port, baudRate);

            // V2: Legacy serial connection removed. Use STM32 Ethernet (Step 8).
            bool success = false;

            return {
                {"success", success},
                {"mode", "SIM"},
                {"port", ""},
                {"driver_name", m_activeDriver->getDriverName()},
                {"error", "Legacy serial connection removed. Use STM32 Ethernet mode."}
            };
        });

    // FIRMWARE_DISCONNECT handler
    m_ipcServer->registerHandler(MessageType::FIRMWARE_DISCONNECT,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("FIRMWARE_DISCONNECT request");

            switchToSimMode();

            return {
                {"success", true},
                {"mode", "SIM"},
                {"driver_name", m_activeDriver->getDriverName()}
            };
        });

    // FIRMWARE_GET_MODE handler
    m_ipcServer->registerHandler(MessageType::FIRMWARE_GET_MODE,
        [this](const Message& request) -> nlohmann::json {
            return {
                {"mode", m_isSimMode ? "SIM" : "STM32"},
                {"is_connected", m_activeDriver->isConnected()},
                {"driver_name", m_activeDriver->getDriverName()},
                {"is_simulation", m_activeDriver->isSimulation()},
                {"port", ""}
            };
        });

    // FIRMWARE_SCAN_PORTS handler — V2: returns empty, serial ports no longer used
    m_ipcServer->registerHandler(MessageType::FIRMWARE_SCAN_PORTS,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("FIRMWARE_SCAN_PORTS request — legacy serial removed");

            return {
                {"ports", nlohmann::json::array()},
                {"count", 0}
            };
        });

    // ========================================================================
    // V2: Drive/Home/STM32 IPC Handlers
    // ========================================================================

    // ENABLE_DRIVES
    m_ipcServer->registerHandler(MessageType::ENABLE_DRIVES,
        [this](const Message& request) -> nlohmann::json {
            uint8_t mask = request.payload.value("axis_mask", 0x3F);
            LOG_INFO("ENABLE_DRIVES request (mask=0x{:02X})", mask);
            bool success = enableDrives(mask);
            return {
                {"success", success},
                {"drives_enabled", success}
            };
        });

    // DISABLE_DRIVES
    m_ipcServer->registerHandler(MessageType::DISABLE_DRIVES,
        [this](const Message& request) -> nlohmann::json {
            uint8_t mask = request.payload.value("axis_mask", 0x3F);
            LOG_INFO("DISABLE_DRIVES request (mask=0x{:02X})", mask);
            bool success = disableDrives(mask);
            return {
                {"success", success},
                {"drives_enabled", false}
            };
        });

    // RESET_ALARM
    m_ipcServer->registerHandler(MessageType::RESET_ALARM,
        [this](const Message& request) -> nlohmann::json {
            uint8_t mask = request.payload.value("axis_mask", 0x3F);
            LOG_INFO("RESET_ALARM request (mask=0x{:02X})", mask);
            bool success = m_activeDriver->resetAlarm(mask);
            return {{"success", success}};
        });

    // HOME_ALL
    m_ipcServer->registerHandler(MessageType::HOME_ALL,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("HOME_ALL request");
            bool success = goHome(0x3F);
            return {
                {"success", success},
                {"homing", success}
            };
        });

    // HOME_AXIS
    m_ipcServer->registerHandler(MessageType::HOME_AXIS,
        [this](const Message& request) -> nlohmann::json {
            int axis = request.payload.value("axis", 0);
            LOG_INFO("HOME_AXIS request (axis={})", axis);
            bool success = goHome(static_cast<uint8_t>(1 << axis));
            return {
                {"success", success},
                {"axis", axis}
            };
        });

    // GET_DRIVE_STATUS
    m_ipcServer->registerHandler(MessageType::GET_DRIVE_STATUS,
        [this](const Message& request) -> nlohmann::json {
            auto pkt = m_activeDriver->getStatusPacket();
            return {
                {"drive_ready", pkt.drive_ready},
                {"drive_alarm", pkt.drive_alarm},
                {"home_status", pkt.home_status},
                {"system_state", firmware::protocol::systemStateToString(
                    static_cast<firmware::protocol::SystemState>(pkt.state))},
                {"buffer_level", pkt.pvt_buffer_lvl}
            };
        });

    // STM32_CONNECT
    m_ipcServer->registerHandler(MessageType::STM32_CONNECT,
        [this](const Message& request) -> nlohmann::json {
            std::string ip = request.payload.value("ip", "192.168.1.100");
            int port = request.payload.value("port", 5001);
            LOG_INFO("STM32_CONNECT request: ip={} port={}", ip, port);

            bool success = switchToSTM32Mode(ip, static_cast<uint16_t>(port));
            return {
                {"success", success},
                {"mode", success ? "STM32" : "SIM"},
                {"ip", ip},
                {"port", port},
                {"driver_name", m_activeDriver->getDriverName()},
                {"error", success ? "" : "Failed to connect to STM32"}
            };
        });

    // STM32_DISCONNECT
    m_ipcServer->registerHandler(MessageType::STM32_DISCONNECT,
        [this](const Message& request) -> nlohmann::json {
            LOG_INFO("STM32_DISCONNECT request");
            switchToSimMode();
            return {
                {"success", true},
                {"mode", "SIM"},
                {"driver_name", m_activeDriver->getDriverName()}
            };
        });

    // GET_IO_STATE
    m_ipcServer->registerHandler(MessageType::GET_IO_STATE,
        [this](const Message& request) -> nlohmann::json {
            return {
                {"digital_inputs", m_activeDriver->getDigitalInputs()},
                {"digital_outputs", m_activeDriver->getDigitalOutputs()}
            };
        });

    // SET_IO_OUTPUT
    m_ipcServer->registerHandler(MessageType::SET_IO_OUTPUT,
        [this](const Message& request) -> nlohmann::json {
            int index = request.payload.value("index", -1);
            bool value = request.payload.value("value", false);

            if (index < 0 || index > 15) {
                return {{"success", false}, {"error", "Invalid I/O index"}};
            }

            bool success = m_activeDriver->setOutput(static_cast<uint8_t>(index), value);
            return {{"success", success}};
        });

    LOG_DEBUG("IPC handlers registered");
}

} // namespace robot_controller
