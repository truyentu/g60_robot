/**
 * @file HomingService.hpp
 * @brief Service for managing robot homing/mastering sequences
 */

#pragma once

#include "HomingTypes.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace robot_controller {
namespace homing {

/**
 * @brief Interface for motor control during homing
 *
 * This interface abstracts the motion controller for homing operations.
 * The actual implementation will be provided by MotionController.
 */
class IHomingMotionInterface {
public:
    virtual ~IHomingMotionInterface() = default;

    /// Move joint at constant velocity
    virtual bool moveJointVelocity(int jointIndex, double velocity) = 0;

    /// Stop joint motion
    virtual bool stopJoint(int jointIndex) = 0;

    /// Stop all joints
    virtual bool stopAllJoints() = 0;

    /// Get current joint position
    virtual double getJointPosition(int jointIndex) const = 0;

    /// Check if limit switch is active for joint
    virtual bool isLimitSwitchActive(int jointIndex) const = 0;

    /// Set joint position to zero (or offset)
    virtual bool setJointZero(int jointIndex, double offset = 0.0) = 0;

    /// Get number of joints
    virtual int getJointCount() const = 0;
};

/**
 * @brief Service for managing robot homing sequences
 *
 * Implements a state machine for homing each joint:
 * 1. IDLE -> MOVING_TO_SWITCH: Start moving toward limit switch
 * 2. MOVING_TO_SWITCH -> BACKING_OFF: Hit switch, back off
 * 3. BACKING_OFF -> MOVING_SLOW: Move slowly toward switch
 * 4. MOVING_SLOW -> SETTING_ZERO: Hit switch at slow speed
 * 5. SETTING_ZERO -> COMPLETE: Set zero position
 */
class HomingService {
public:
    static constexpr int MAX_JOINTS = 6;

    /**
     * @brief Constructor
     * @param motionInterface Interface to motion controller
     */
    explicit HomingService(std::shared_ptr<IHomingMotionInterface> motionInterface = nullptr);

    /**
     * @brief Destructor - stops any active homing
     */
    ~HomingService();

    // Non-copyable
    HomingService(const HomingService&) = delete;
    HomingService& operator=(const HomingService&) = delete;

    // ========================================================================
    // Motion Interface
    // ========================================================================

    /**
     * @brief Set the motion interface
     * @param motionInterface Interface to motion controller
     */
    void setMotionInterface(std::shared_ptr<IHomingMotionInterface> motionInterface);

    // ========================================================================
    // Homing Control
    // ========================================================================

    /**
     * @brief Start homing for specific joint or all joints
     * @param jointIndex Joint to home (-1 for all joints)
     * @param method Homing method to use (optional, uses configured method if not specified)
     * @return true if homing started successfully
     */
    bool startHoming(int jointIndex = -1, HomingMethod method = HomingMethod::LIMIT_SWITCH);

    /**
     * @brief Stop homing for specific joint or all joints
     * @param jointIndex Joint to stop (-1 for all joints)
     * @return true if stop command sent successfully
     */
    bool stopHoming(int jointIndex = -1);

    /**
     * @brief Mark joint as manually homed (for MANUAL method)
     * @param jointIndex Joint to mark as homed
     * @return true if successful
     */
    bool markAsHomed(int jointIndex);

    // ========================================================================
    // Status Queries
    // ========================================================================

    /**
     * @brief Get homing state for a specific joint
     */
    HomingState getJointHomingState(int jointIndex) const;

    /**
     * @brief Get detailed status for a specific joint
     */
    JointHomingStatus getJointStatus(int jointIndex) const;

    /**
     * @brief Get status for all joints
     */
    HomingSystemStatus getSystemStatus() const;

    /**
     * @brief Check if all joints are homed
     */
    bool isAllJointsHomed() const;

    /**
     * @brief Check if any joint is currently homing
     */
    bool isAnyJointHoming() const;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set homing configuration for a joint
     */
    void setHomingConfig(int jointIndex, const HomingConfig& config);

    /**
     * @brief Get homing configuration for a joint
     */
    HomingConfig getHomingConfig(int jointIndex) const;

    /**
     * @brief Load homing configuration from YAML file
     */
    bool loadConfig(const std::string& path);

    /**
     * @brief Save homing configuration to YAML file
     */
    bool saveConfig(const std::string& path) const;

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Set callback for homing state changes
     */
    void setHomingCallback(HomingStateCallback callback);

private:
    // Motion interface
    std::shared_ptr<IHomingMotionInterface> m_motionInterface;

    // Configuration for each joint
    std::array<HomingConfig, MAX_JOINTS> m_configs;

    // Current status for each joint
    std::array<JointHomingStatus, MAX_JOINTS> m_status;

    // Callback for state changes
    HomingStateCallback m_callback;

    // Thread safety
    mutable std::mutex m_mutex;

    // Homing thread
    std::unique_ptr<std::thread> m_homingThread;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_isRunning{false};

    // Joints currently being homed
    std::vector<int> m_jointsToHome;

    // Private methods
    void homingThreadFunc();
    void executeHomingSequence(int jointIndex);
    void updateJointState(int jointIndex, HomingState state, const std::string& error = "");
    void updateSequenceState(int jointIndex, HomingSequenceState seqState);
    bool waitForCondition(std::function<bool()> condition, double timeoutSec);
};

} // namespace homing
} // namespace robot_controller
