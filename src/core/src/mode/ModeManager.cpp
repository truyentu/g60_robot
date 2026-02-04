#include "ModeManager.hpp"
#include "../logging/Logger.hpp"

namespace robot_controller {
namespace mode {

ModeManager::ModeManager() {
    LOG_INFO("ModeManager initialized in MANUAL mode");
}

OperationMode ModeManager::getCurrentMode() const {
    return m_currentMode.load();
}

state::RobotMode ModeManager::getRobotMode() const {
    // Map OperationMode to state::RobotMode
    switch (m_currentMode.load()) {
        case OperationMode::MANUAL:
            return state::RobotMode::T1;  // T1 = Teaching mode with reduced speed
        case OperationMode::TEST:
            return state::RobotMode::T2;  // T2 = Test mode with full speed
        case OperationMode::AUTO:
            return state::RobotMode::AUTO;
        case OperationMode::REMOTE:
            return state::RobotMode::AUTO;  // Remote uses AUTO mode with external control
        default:
            return state::RobotMode::MANUAL;
    }
}

bool ModeManager::requestModeChange(OperationMode newMode) {
    std::lock_guard<std::mutex> lock(m_mutex);

    OperationMode currentMode = m_currentMode.load();

    if (currentMode == newMode) {
        LOG_DEBUG("Already in {} mode", operationModeToString(newMode));
        return true;
    }

    // Check requirements
    if (!checkTransitionRequirements(newMode)) {
        auto missing = getMissingRequirements(newMode);
        LOG_WARN("Cannot transition to {}, missing requirements:",
                    operationModeToString(newMode));
        for (const auto& req : missing) {
            LOG_WARN("  - {}", req);
        }
        return false;
    }

    // Perform transition
    OperationMode oldMode = currentMode;
    m_currentMode.store(newMode);

    LOG_INFO("Mode changed: {} -> {}",
             operationModeToString(oldMode),
             operationModeToString(newMode));

    // Apply configuration
    applyModeConfig(newMode);

    // Fire callback
    if (m_callback) {
        m_callback(oldMode, newMode);
    }

    return true;
}

bool ModeManager::canTransitionTo(OperationMode newMode) const {
    return checkTransitionRequirements(newMode);
}

ModeTransitionRequirement ModeManager::getTransitionRequirements(OperationMode target) const {
    ModeTransitionRequirement req;

    // Base requirements for all transitions
    req.robotStopped = true;      // Robot must be stopped
    req.noActiveAlarms = true;    // No active alarms

    // Mode-specific requirements
    switch (target) {
        case OperationMode::MANUAL:
            req.safetyFenceClosed = false;  // Fence not required for teaching
            req.deadmanReleased = true;     // Deadman must be released before switching
            req.homingComplete = false;     // Can enter MANUAL without homing
            break;

        case OperationMode::TEST:
            req.safetyFenceClosed = false;  // Fence not required
            req.deadmanReleased = true;
            req.homingComplete = true;      // Must be homed
            break;

        case OperationMode::AUTO:
            req.safetyFenceClosed = true;   // Safety fence required
            req.deadmanReleased = true;
            req.homingComplete = true;      // Must be homed
            break;

        case OperationMode::REMOTE:
            req.safetyFenceClosed = true;   // Safety fence required
            req.deadmanReleased = true;
            req.homingComplete = true;      // Must be homed
            break;
    }

    return req;
}

std::vector<std::string> ModeManager::getMissingRequirements(OperationMode target) const {
    std::vector<std::string> missing;
    auto req = getTransitionRequirements(target);

    if (req.robotStopped && !m_robotStopped.load()) {
        missing.push_back("Robot must be stopped");
    }

    if (req.safetyFenceClosed && !m_safetyFenceClosed.load()) {
        missing.push_back("Safety fence must be closed");
    }

    if (req.deadmanReleased && m_deadmanPressed.load()) {
        missing.push_back("Deadman switch must be released");
    }

    if (req.noActiveAlarms && m_hasAlarms.load()) {
        missing.push_back("All alarms must be cleared");
    }

    if (req.homingComplete && !m_homingComplete.load()) {
        missing.push_back("Homing must be completed");
    }

    return missing;
}

OperationModeConfig ModeManager::getCurrentModeConfig() const {
    auto it = MODE_CONFIGS.find(m_currentMode.load());
    if (it != MODE_CONFIGS.end()) {
        return it->second;
    }
    // Default to MANUAL config if not found
    return MODE_CONFIGS.at(OperationMode::MANUAL);
}

double ModeManager::getMaxLinearVelocity() const {
    return getCurrentModeConfig().maxLinearVelocity;
}

double ModeManager::getMaxJointVelocity() const {
    return getCurrentModeConfig().maxJointVelocity;
}

bool ModeManager::isDeadmanRequired() const {
    return getCurrentModeConfig().requireDeadman;
}

bool ModeManager::isSafetyFenceRequired() const {
    return getCurrentModeConfig().requireSafetyFence;
}

bool ModeManager::isExternalControlAllowed() const {
    return getCurrentModeConfig().allowExternalControl;
}

void ModeManager::setModeChangeCallback(ModeChangeCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callback = std::move(callback);
}

void ModeManager::updateRobotStopped(bool stopped) {
    m_robotStopped.store(stopped);
}

void ModeManager::updateSafetyFence(bool closed) {
    m_safetyFenceClosed.store(closed);
}

void ModeManager::updateDeadman(bool pressed) {
    m_deadmanPressed.store(pressed);
}

void ModeManager::updateAlarms(bool hasAlarms) {
    m_hasAlarms.store(hasAlarms);
}

void ModeManager::updateHomingComplete(bool complete) {
    m_homingComplete.store(complete);
}

bool ModeManager::checkTransitionRequirements(OperationMode target) const {
    auto missing = getMissingRequirements(target);
    return missing.empty();
}

void ModeManager::applyModeConfig(OperationMode mode) {
    auto config = MODE_CONFIGS.at(mode);

    LOG_DEBUG("Applied mode config for {}:", operationModeToString(mode));
    LOG_DEBUG("  Max linear velocity: {} mm/s", config.maxLinearVelocity);
    LOG_DEBUG("  Max joint velocity: {} deg/s", config.maxJointVelocity);
    LOG_DEBUG("  Deadman required: {}", config.requireDeadman);
    LOG_DEBUG("  Safety fence required: {}", config.requireSafetyFence);
    LOG_DEBUG("  External control allowed: {}", config.allowExternalControl);
}

} // namespace mode
} // namespace robot_controller
