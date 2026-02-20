/**
 * @file RobotController.hpp
 * @brief Main robot controller class
 */

#pragma once

#include "../state/StateMachine.hpp"
#include "../config/ConfigManager.hpp"
#include "../ipc/IpcServer.hpp"
#include "../frame/BaseFrameManager.hpp"
#include "../override/OverrideManager.hpp"
#include "../interpreter/Executor.hpp"
#include "../interpreter/Lexer.hpp"
#include "../interpreter/Parser.hpp"
#include "../jog/JogController.hpp"
#include "../firmware/IFirmwareDriver.hpp"
#include "../firmware/FirmwareSimulator.hpp"
#include "../config/RobotPackageSchema.hpp"
#include "../tool/ToolManager.hpp"
#include <memory>
#include <array>
#include <atomic>
#include <thread>
#include <queue>

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
    std::array<double, 6> tcpPose = {0};           // X,Y,Z,Rx,Ry,Rz (World frame)
    std::array<double, 6> tcpInBase = {0};         // X,Y,Z,Rx,Ry,Rz (Active base frame)

    std::string activeBaseId = "world";            // Active base frame ID
    std::string activeToolId = "tool_default";     // Active tool ID

    state::ErrorCode errorCode = state::ErrorCode::NONE;
    std::string errorMessage;

    // Triple override control (KUKA-inspired)
    int programOverride = 100;    // Program execution speed (1-100%)
    int jogOverride = 100;        // Manual jog speed (1-100%)
    int manualOverride = 100;     // T1 mode cap (1-100%, max 250mm/s)

    // V2 additions
    uint8_t driveReady = 0;         // bit0-5: drive ready flags
    uint8_t driveAlarm = 0;         // bit0-5: drive alarm flags
    uint8_t bufferLevel = 0;        // PVT buffer fill (0-255)
    uint16_t digitalInputs = 0;    // Digital inputs state
    uint16_t digitalOutputs = 0;   // Digital outputs state
    uint8_t homeStatus = 0;         // bit0-5: axis homed flags
    std::string firmwareMode = "SIM";
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
     * Set workspace path (call before initialize)
     */
    void setWorkspacePath(const std::string& path) { m_workspacePath = path; }

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

    // Firmware mode switch (Phase 9 → V2)
    bool switchToSimMode();
    bool switchToRealMode(const std::string& portName = "");  // Legacy (returns false)
    bool switchToSTM32Mode(const std::string& ip = "192.168.1.100", uint16_t port = 5001);
    std::string getFirmwareMode() const;

    // V2: Drive/Home control
    bool enableDrives(uint8_t axisMask = 0x3F);
    bool disableDrives(uint8_t axisMask = 0x3F);
    bool goHome(uint8_t axisMask = 0x3F);

    // Program execution (Phase 8)
    bool loadProgram(const std::string& source);
    void runProgram();
    void stepProgram();
    void pauseProgram();
    void stopProgram();
    void resetProgram();
    interpreter::ExecutionState getProgramState() const;
    int getProgramLine() const;
    std::string getProgramName() const;

    // Point management (Phase 8)
    void setPoint(const std::string& name, const std::vector<double>& values);
    std::vector<double> getPoint(const std::string& name) const;

private:
    void controlLoop();
    void updateStatus();
    void publishStatus();
    void registerIpcHandlers();
    void saveBaseFrames();

    // Components
    std::unique_ptr<state::StateMachine> m_stateMachine;
    std::unique_ptr<ipc::IpcServer> m_ipcServer;
    std::unique_ptr<frame::BaseFrameManager> m_baseFrameManager;
    std::unique_ptr<override::OverrideManager> m_overrideManager;
    std::unique_ptr<jog::JogController> m_jogController;
    std::shared_ptr<firmware::IFirmwareDriver> m_activeDriver;
    std::shared_ptr<firmware::FirmwareSimulator> m_simDriver;
    std::shared_ptr<firmware::IFirmwareDriver> m_realDriver;  // V2: STM32EthernetDriver (set in switchToSTM32Mode)
    bool m_isSimMode{true};
    std::optional<config::RobotPackage> m_activePackage;
    Eigen::Matrix4d m_flangeTransform = Eigen::Matrix4d::Identity(); // T_flangeOffset from package
    std::unique_ptr<interpreter::Executor> m_programExecutor;
    std::unique_ptr<tool::ToolManager> m_toolManager;

    // Status
    RobotStatus m_status;
    mutable std::mutex m_statusMutex;

    // Control loop
    std::thread m_controlThread;
    std::atomic<bool> m_running{false};
    int m_cycleTimeMs = 4;  // 250 Hz
    int m_statusPublishHz = 10;

    // Packet log queue (thread-safe: IO thread pushes, control loop publishes)
    std::queue<ipc::Message> m_packetLogQueue;
    std::mutex m_packetLogMutex;

    // Workspace path (optional, takes priority over configDir)
    std::string m_workspacePath;
};

} // namespace robot_controller
