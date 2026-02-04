/**
 * @file HomingService.cpp
 * @brief Implementation of robot homing/mastering service
 */

#include "HomingService.hpp"
#include <spdlog/spdlog.h>
#include <chrono>

namespace robot_controller {
namespace homing {

HomingService::HomingService(std::shared_ptr<IHomingMotionInterface> motionInterface)
    : m_motionInterface(motionInterface)
{
    // Initialize status for all joints
    for (int i = 0; i < MAX_JOINTS; ++i) {
        m_status[i].jointIndex = i;
        m_status[i].state = HomingState::NOT_HOMED;
        m_status[i].sequenceState = HomingSequenceState::IDLE;
    }

    spdlog::info("HomingService initialized");
}

HomingService::~HomingService()
{
    stopHoming(-1);

    // Wait for thread to finish
    if (m_homingThread && m_homingThread->joinable()) {
        m_homingThread->join();
    }
}

void HomingService::setMotionInterface(std::shared_ptr<IHomingMotionInterface> motionInterface)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_motionInterface = motionInterface;
}

// ============================================================================
// Homing Control
// ============================================================================

bool HomingService::startHoming(int jointIndex, HomingMethod method)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_motionInterface) {
        spdlog::error("HomingService: No motion interface configured");
        return false;
    }

    if (m_isRunning) {
        spdlog::warn("HomingService: Homing already in progress");
        return false;
    }

    // Determine which joints to home
    m_jointsToHome.clear();
    int jointCount = m_motionInterface->getJointCount();

    if (jointIndex < 0) {
        // Home all joints
        for (int i = 0; i < jointCount && i < MAX_JOINTS; ++i) {
            m_jointsToHome.push_back(i);
            m_configs[i].method = method;
        }
        spdlog::info("HomingService: Starting homing for all {} joints", jointCount);
    } else if (jointIndex < jointCount && jointIndex < MAX_JOINTS) {
        m_jointsToHome.push_back(jointIndex);
        m_configs[jointIndex].method = method;
        spdlog::info("HomingService: Starting homing for joint {}", jointIndex);
    } else {
        spdlog::error("HomingService: Invalid joint index {}", jointIndex);
        return false;
    }

    // Reset states for joints being homed
    for (int j : m_jointsToHome) {
        m_status[j].state = HomingState::HOMING_IN_PROGRESS;
        m_status[j].sequenceState = HomingSequenceState::IDLE;
        m_status[j].progress = 0.0;
        m_status[j].errorMessage.clear();
    }

    // Start homing thread
    m_stopRequested = false;
    m_isRunning = true;

    if (m_homingThread && m_homingThread->joinable()) {
        m_homingThread->join();
    }
    m_homingThread = std::make_unique<std::thread>(&HomingService::homingThreadFunc, this);

    return true;
}

bool HomingService::stopHoming(int jointIndex)
{
    spdlog::info("HomingService: Stop requested for joint {}", jointIndex);

    m_stopRequested = true;

    if (m_motionInterface) {
        if (jointIndex < 0) {
            m_motionInterface->stopAllJoints();
        } else {
            m_motionInterface->stopJoint(jointIndex);
        }
    }

    // Wait for thread to finish
    if (m_homingThread && m_homingThread->joinable()) {
        m_homingThread->join();
    }

    // Update states
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int j : m_jointsToHome) {
        if (m_status[j].state == HomingState::HOMING_IN_PROGRESS) {
            m_status[j].state = HomingState::NOT_HOMED;
            m_status[j].sequenceState = HomingSequenceState::IDLE;
        }
    }
    m_jointsToHome.clear();
    m_isRunning = false;

    return true;
}

bool HomingService::markAsHomed(int jointIndex)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (jointIndex < 0 || jointIndex >= MAX_JOINTS) {
        return false;
    }

    m_status[jointIndex].state = HomingState::HOMED;
    m_status[jointIndex].sequenceState = HomingSequenceState::COMPLETE;
    m_status[jointIndex].progress = 1.0;

    if (m_motionInterface) {
        m_motionInterface->setJointZero(jointIndex, m_configs[jointIndex].homeOffset);
    }

    spdlog::info("HomingService: Joint {} marked as manually homed", jointIndex);

    if (m_callback) {
        m_callback(jointIndex, HomingState::HOMED);
    }

    return true;
}

// ============================================================================
// Status Queries
// ============================================================================

HomingState HomingService::getJointHomingState(int jointIndex) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (jointIndex < 0 || jointIndex >= MAX_JOINTS) {
        return HomingState::NOT_HOMED;
    }
    return m_status[jointIndex].state;
}

JointHomingStatus HomingService::getJointStatus(int jointIndex) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (jointIndex < 0 || jointIndex >= MAX_JOINTS) {
        return JointHomingStatus{};
    }
    return m_status[jointIndex];
}

HomingSystemStatus HomingService::getSystemStatus() const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    HomingSystemStatus status;
    int jointCount = m_motionInterface ? m_motionInterface->getJointCount() : MAX_JOINTS;
    status.totalJoints = jointCount;

    for (int i = 0; i < jointCount && i < MAX_JOINTS; ++i) {
        status.joints.push_back(m_status[i]);

        if (m_status[i].state == HomingState::HOMED) {
            status.homedCount++;
        }
        if (m_status[i].state == HomingState::HOMING_IN_PROGRESS) {
            status.anyHoming = true;
        }
        if (m_status[i].state == HomingState::HOMING_ERROR) {
            status.anyError = true;
        }
    }

    status.allHomed = (status.homedCount == status.totalJoints);

    return status;
}

bool HomingService::isAllJointsHomed() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    int jointCount = m_motionInterface ? m_motionInterface->getJointCount() : MAX_JOINTS;

    for (int i = 0; i < jointCount && i < MAX_JOINTS; ++i) {
        if (m_status[i].state != HomingState::HOMED) {
            return false;
        }
    }
    return true;
}

bool HomingService::isAnyJointHoming() const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (int i = 0; i < MAX_JOINTS; ++i) {
        if (m_status[i].state == HomingState::HOMING_IN_PROGRESS) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// Configuration
// ============================================================================

void HomingService::setHomingConfig(int jointIndex, const HomingConfig& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (jointIndex >= 0 && jointIndex < MAX_JOINTS) {
        m_configs[jointIndex] = config;
    }
}

HomingConfig HomingService::getHomingConfig(int jointIndex) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (jointIndex >= 0 && jointIndex < MAX_JOINTS) {
        return m_configs[jointIndex];
    }
    return HomingConfig{};
}

bool HomingService::loadConfig(const std::string& path)
{
    // TODO: Implement YAML loading
    spdlog::info("HomingService: Loading config from {}", path);
    return true;
}

bool HomingService::saveConfig(const std::string& path) const
{
    // TODO: Implement YAML saving
    spdlog::info("HomingService: Saving config to {}", path);
    return true;
}

// ============================================================================
// Callbacks
// ============================================================================

void HomingService::setHomingCallback(HomingStateCallback callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callback = callback;
}

// ============================================================================
// Private Methods
// ============================================================================

void HomingService::homingThreadFunc()
{
    spdlog::info("HomingService: Homing thread started");

    // Home each joint sequentially
    for (int jointIndex : m_jointsToHome) {
        if (m_stopRequested) {
            break;
        }

        executeHomingSequence(jointIndex);
    }

    m_isRunning = false;
    spdlog::info("HomingService: Homing thread finished");
}

void HomingService::executeHomingSequence(int jointIndex)
{
    auto& config = m_configs[jointIndex];
    auto& status = m_status[jointIndex];

    spdlog::info("HomingService: Executing homing sequence for joint {}, method: {}",
                 jointIndex, homingMethodToString(config.method));

    // Handle different homing methods
    switch (config.method) {
        case HomingMethod::ABSOLUTE_ENCODER:
            // Absolute encoder just needs validation
            updateSequenceState(jointIndex, HomingSequenceState::SETTING_ZERO);
            status.progress = 0.9;
            m_motionInterface->setJointZero(jointIndex, config.homeOffset);
            updateJointState(jointIndex, HomingState::HOMED);
            updateSequenceState(jointIndex, HomingSequenceState::COMPLETE);
            status.progress = 1.0;
            return;

        case HomingMethod::MANUAL:
            // Manual homing - wait for operator to call markAsHomed()
            spdlog::info("HomingService: Joint {} waiting for manual homing confirmation", jointIndex);
            return;

        case HomingMethod::LIMIT_SWITCH:
        case HomingMethod::INDEX_PULSE:
            // Continue with standard homing sequence below
            break;
    }

    // Standard limit switch homing sequence
    auto startTime = std::chrono::steady_clock::now();

    // Determine velocity direction
    double velocity = config.searchVelocity;
    if (config.direction == HomingDirection::NEGATIVE) {
        velocity = -velocity;
    }

    // Phase 1: Move toward limit switch (fast)
    updateSequenceState(jointIndex, HomingSequenceState::MOVING_TO_SWITCH);
    status.progress = 0.1;

    if (!m_motionInterface->moveJointVelocity(jointIndex, velocity)) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Failed to start motion");
        return;
    }

    // Wait for limit switch
    bool switchFound = waitForCondition([&]() {
        status.currentPosition = m_motionInterface->getJointPosition(jointIndex);
        status.limitSwitchActive = m_motionInterface->isLimitSwitchActive(jointIndex);
        return status.limitSwitchActive || m_stopRequested;
    }, config.timeout);

    m_motionInterface->stopJoint(jointIndex);

    if (m_stopRequested) {
        updateJointState(jointIndex, HomingState::NOT_HOMED, "Homing cancelled");
        return;
    }

    if (!switchFound) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Limit switch not found (timeout)");
        return;
    }

    spdlog::info("HomingService: Joint {} found limit switch at position {}",
                 jointIndex, status.currentPosition);

    // Phase 2: Back off from switch
    updateSequenceState(jointIndex, HomingSequenceState::BACKING_OFF);
    status.progress = 0.4;

    double backoffVelocity = -velocity; // Opposite direction
    double targetBackoff = status.currentPosition + (config.direction == HomingDirection::NEGATIVE
                                                      ? config.backoffDistance
                                                      : -config.backoffDistance);

    if (!m_motionInterface->moveJointVelocity(jointIndex, backoffVelocity * 0.5)) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Failed to back off");
        return;
    }

    // Wait until off switch and moved backoff distance
    waitForCondition([&]() {
        status.currentPosition = m_motionInterface->getJointPosition(jointIndex);
        status.limitSwitchActive = m_motionInterface->isLimitSwitchActive(jointIndex);
        bool movedEnough = std::abs(status.currentPosition - targetBackoff) < 0.5;
        return (!status.limitSwitchActive && movedEnough) || m_stopRequested;
    }, 10.0);

    m_motionInterface->stopJoint(jointIndex);

    if (m_stopRequested) {
        updateJointState(jointIndex, HomingState::NOT_HOMED, "Homing cancelled");
        return;
    }

    // Phase 3: Move slowly toward switch
    updateSequenceState(jointIndex, HomingSequenceState::MOVING_SLOW);
    status.progress = 0.7;

    double slowVelocity = config.locateVelocity;
    if (config.direction == HomingDirection::NEGATIVE) {
        slowVelocity = -slowVelocity;
    }

    if (!m_motionInterface->moveJointVelocity(jointIndex, slowVelocity)) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Failed to start slow motion");
        return;
    }

    // Wait for limit switch at slow speed
    switchFound = waitForCondition([&]() {
        status.currentPosition = m_motionInterface->getJointPosition(jointIndex);
        status.limitSwitchActive = m_motionInterface->isLimitSwitchActive(jointIndex);
        return status.limitSwitchActive || m_stopRequested;
    }, config.timeout);

    m_motionInterface->stopJoint(jointIndex);

    if (m_stopRequested) {
        updateJointState(jointIndex, HomingState::NOT_HOMED, "Homing cancelled");
        return;
    }

    if (!switchFound) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Limit switch not found on slow approach");
        return;
    }

    // Phase 4: Set zero position
    updateSequenceState(jointIndex, HomingSequenceState::SETTING_ZERO);
    status.progress = 0.9;

    if (!m_motionInterface->setJointZero(jointIndex, config.homeOffset)) {
        updateJointState(jointIndex, HomingState::HOMING_ERROR, "Failed to set zero position");
        return;
    }

    // Complete
    updateSequenceState(jointIndex, HomingSequenceState::COMPLETE);
    status.progress = 1.0;
    updateJointState(jointIndex, HomingState::HOMED);

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startTime);
    spdlog::info("HomingService: Joint {} homing complete in {} ms", jointIndex, elapsed.count());
}

void HomingService::updateJointState(int jointIndex, HomingState state, const std::string& error)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status[jointIndex].state = state;
        m_status[jointIndex].errorMessage = error;
    }

    if (state == HomingState::HOMING_ERROR) {
        spdlog::error("HomingService: Joint {} error: {}", jointIndex, error);
    }

    if (m_callback) {
        m_callback(jointIndex, state);
    }
}

void HomingService::updateSequenceState(int jointIndex, HomingSequenceState seqState)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status[jointIndex].sequenceState = seqState;
}

bool HomingService::waitForCondition(std::function<bool()> condition, double timeoutSec)
{
    auto startTime = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(timeoutSec);

    while (!condition()) {
        if (std::chrono::steady_clock::now() - startTime > timeout) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}

} // namespace homing
} // namespace robot_controller
