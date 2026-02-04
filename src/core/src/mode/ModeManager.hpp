#pragma once

#include "OperationMode.hpp"
#include "../state/States.hpp"
#include <functional>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>

namespace robot_controller {
namespace mode {

/**
 * @brief KUKA-inspired Operation Mode Manager
 *
 * Manages mode transitions with safety requirements and velocity limits.
 * Integrates with existing RobotController state machine.
 */
class ModeManager {
public:
    ModeManager();
    ~ModeManager() = default;

    // Non-copyable
    ModeManager(const ModeManager&) = delete;
    ModeManager& operator=(const ModeManager&) = delete;

    /**
     * @brief Get current operation mode
     */
    OperationMode getCurrentMode() const;

    /**
     * @brief Get corresponding RobotMode for state machine
     */
    state::RobotMode getRobotMode() const;

    /**
     * @brief Request mode change
     * @param newMode Target operation mode
     * @return true if transition successful
     */
    bool requestModeChange(OperationMode newMode);

    /**
     * @brief Check if transition to mode is possible
     * @param newMode Target mode
     * @return true if all requirements met
     */
    bool canTransitionTo(OperationMode newMode) const;

    /**
     * @brief Get requirements for transitioning to a mode
     * @param target Target mode
     * @return Requirements structure
     */
    ModeTransitionRequirement getTransitionRequirements(OperationMode target) const;

    /**
     * @brief Get list of missing requirements for a mode
     * @param target Target mode
     * @return List of requirement descriptions that are not met
     */
    std::vector<std::string> getMissingRequirements(OperationMode target) const;

    /**
     * @brief Get current mode configuration
     */
    OperationModeConfig getCurrentModeConfig() const;

    /**
     * @brief Get maximum linear velocity for current mode
     * @return Velocity in mm/s
     */
    double getMaxLinearVelocity() const;

    /**
     * @brief Get maximum joint velocity for current mode
     * @return Velocity in deg/s
     */
    double getMaxJointVelocity() const;

    /**
     * @brief Check if deadman switch is required in current mode
     */
    bool isDeadmanRequired() const;

    /**
     * @brief Check if safety fence is required in current mode
     */
    bool isSafetyFenceRequired() const;

    /**
     * @brief Check if external control (PLC) is allowed in current mode
     */
    bool isExternalControlAllowed() const;

    // Mode change callback
    using ModeChangeCallback = std::function<void(OperationMode oldMode, OperationMode newMode)>;

    /**
     * @brief Set callback for mode change events
     */
    void setModeChangeCallback(ModeChangeCallback callback);

    // System state updates (called by RobotController)
    void updateRobotStopped(bool stopped);
    void updateSafetyFence(bool closed);
    void updateDeadman(bool pressed);
    void updateAlarms(bool hasAlarms);
    void updateHomingComplete(bool complete);

private:
    /**
     * @brief Check if all transition requirements are met
     */
    bool checkTransitionRequirements(OperationMode target) const;

    /**
     * @brief Apply mode configuration after successful transition
     */
    void applyModeConfig(OperationMode mode);

    // Current mode
    std::atomic<OperationMode> m_currentMode{OperationMode::MANUAL};

    // System state (updated externally)
    std::atomic<bool> m_robotStopped{true};
    std::atomic<bool> m_safetyFenceClosed{false};
    std::atomic<bool> m_deadmanPressed{false};
    std::atomic<bool> m_hasAlarms{false};
    std::atomic<bool> m_homingComplete{false};

    // Callback
    ModeChangeCallback m_callback;
    mutable std::mutex m_mutex;
};

} // namespace mode
} // namespace robot_controller
