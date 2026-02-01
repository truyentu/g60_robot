/**
 * @file RobotController.hpp
 * @brief Main robot controller class
 */

#pragma once

#include "../state/StateMachine.hpp"
#include "../config/ConfigManager.hpp"
#include "../ipc/IpcServer.hpp"
#include <memory>
#include <array>
#include <atomic>
#include <thread>

namespace robot_controller {

/**
 * Robot status data
 */
struct RobotStatus {
    state::RobotState state = state::RobotState::INIT;
    state::RobotMode mode = state::RobotMode::MANUAL;
    state::SafetyState safety = state::SafetyState::NORMAL;

    bool enabled = false;
    bool homed = false;

    std::array<double, 6> jointPositions = {0};    // degrees
    std::array<double, 6> jointVelocities = {0};   // deg/s
    std::array<double, 6> tcpPose = {0};           // X,Y,Z,Rx,Ry,Rz

    state::ErrorCode errorCode = state::ErrorCode::NONE;
    std::string errorMessage;

    double speedOverride = 100.0;  // 0-100%
};

/**
 * Main Robot Controller
 *
 * Coordinates state machine, motion, and IPC
 */
class RobotController {
public:
    RobotController();
    ~RobotController();

    // Non-copyable
    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;

    /**
     * Initialize the controller
     */
    bool initialize(const std::string& configDir = "config");

    /**
     * Start the control loop
     */
    bool start();

    /**
     * Stop the controller
     */
    void stop();

    /**
     * Check if running
     */
    bool isRunning() const { return m_running; }

    /**
     * Get current status
     */
    RobotStatus getStatus() const;

    /**
     * Get state machine
     */
    state::IStateMachine& stateMachine() { return *m_stateMachine; }

    // Commands
    bool enable();
    bool disable();
    bool reset();
    bool home();
    bool setMode(state::RobotMode mode);
    bool setSpeedOverride(double percent);

    // Jog commands (Phase 2)
    bool jogJoint(int joint, double speed);
    bool jogCartesian(int axis, double speed);
    bool stopJog();

private:
    void controlLoop();
    void updateStatus();
    void publishStatus();
    void registerIpcHandlers();

    // Components
    std::unique_ptr<state::StateMachine> m_stateMachine;
    std::unique_ptr<ipc::IpcServer> m_ipcServer;

    // Status
    RobotStatus m_status;
    mutable std::mutex m_statusMutex;

    // Control loop
    std::thread m_controlThread;
    std::atomic<bool> m_running{false};
    int m_cycleTimeMs = 4;  // 250 Hz
    int m_statusPublishHz = 10;
};

} // namespace robot_controller
