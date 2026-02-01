/**
 * @file IStateMachine.hpp
 * @brief State machine interface
 */

#pragma once

#include "States.hpp"
#include <functional>
#include <string>

namespace robot_controller {
namespace state {

/**
 * State change callback signature
 */
using StateChangeCallback = std::function<void(RobotState oldState, RobotState newState)>;

/**
 * Error callback signature
 */
using ErrorCallback = std::function<void(ErrorCode code, const std::string& message)>;

/**
 * State machine interface
 */
class IStateMachine {
public:
    virtual ~IStateMachine() = default;

    // State queries
    virtual RobotState currentState() const = 0;
    virtual RobotMode currentMode() const = 0;
    virtual SafetyState safetyState() const = 0;
    virtual bool isEnabled() const = 0;
    virtual bool isHomed() const = 0;
    virtual bool isMoving() const = 0;
    virtual bool hasError() const = 0;
    virtual ErrorCode lastError() const = 0;
    virtual std::string lastErrorMessage() const = 0;

    // State control
    virtual bool processEvent(StateEvent event) = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool reset() = 0;

    // Mode control
    virtual bool setMode(RobotMode mode) = 0;

    // Error handling
    virtual void setError(ErrorCode code, const std::string& message = "") = 0;
    virtual void clearError() = 0;

    // Callbacks
    virtual void setStateChangeCallback(StateChangeCallback callback) = 0;
    virtual void setErrorCallback(ErrorCallback callback) = 0;

    // State validation
    virtual bool canTransitionTo(RobotState targetState) const = 0;
    virtual bool canExecuteMotion() const = 0;
    virtual bool canJog() const = 0;
    virtual bool canRunProgram() const = 0;
};

} // namespace state
} // namespace robot_controller
