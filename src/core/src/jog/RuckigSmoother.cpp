#include "RuckigSmoother.hpp"
#include "../logging/Logger.hpp"
#include <cmath>

namespace robot_controller {
namespace jog {

RuckigSmoother::RuckigSmoother(double cycleTimeSeconds)
    : m_otg(cycleTimeSeconds)
{
    // Default limits (KUKA KR10 R1420)
    for (int i = 0; i < 6; i++) {
        m_input.max_velocity[i] = 180.0;     // deg/s
        m_input.max_acceleration[i] = 360.0;  // deg/s^2
        m_input.max_jerk[i] = 1000.0;         // deg/s^3
    }
    LOG_DEBUG("RuckigSmoother created (cycle={}s)", cycleTimeSeconds);
}

void RuckigSmoother::setJointLimits(int joint, double maxVel, double maxAccel, double maxJerk) {
    if (joint >= 0 && joint < 6) {
        m_input.max_velocity[joint] = maxVel;
        m_input.max_acceleration[joint] = maxAccel;
        m_input.max_jerk[joint] = maxJerk;
    }
}

void RuckigSmoother::setAllLimits(
    const std::array<double, 6>& maxVelocities,
    const std::array<double, 6>& maxAccelerations,
    const std::array<double, 6>& maxJerks)
{
    for (int i = 0; i < 6; i++) {
        m_input.max_velocity[i] = maxVelocities[i];
        m_input.max_acceleration[i] = maxAccelerations[i];
        m_input.max_jerk[i] = maxJerks[i];
    }
}

bool RuckigSmoother::setTarget(
    const std::array<double, 6>& currentPos,
    const std::array<double, 6>& currentVel,
    const std::array<double, 6>& targetPos,
    const std::array<double, 6>& targetVel)
{
    for (int i = 0; i < 6; i++) {
        m_input.current_position[i] = currentPos[i];
        m_input.current_velocity[i] = currentVel[i];
        m_input.current_acceleration[i] = 0.0;
        m_input.target_position[i] = targetPos[i];
        m_input.target_velocity[i] = targetVel[i];
    }

    m_active = true;
    LOG_DEBUG("RuckigSmoother: New target set");
    return true;
}

RuckigSmoother::TrajectoryPoint RuckigSmoother::update() {
    TrajectoryPoint point;
    point.isComplete = true;

    if (!m_active) {
        for (int i = 0; i < 6; i++) {
            point.position[i] = m_input.current_position[i];
            point.velocity[i] = 0.0;
            point.acceleration[i] = 0.0;
        }
        return point;
    }

    auto result = m_otg.update(m_input, m_output);

    for (int i = 0; i < 6; i++) {
        point.position[i] = m_output.new_position[i];
        point.velocity[i] = m_output.new_velocity[i];
        point.acceleration[i] = m_output.new_acceleration[i];
    }

    if (result == ruckig::Result::Finished) {
        point.isComplete = true;
        m_active = false;
        LOG_DEBUG("RuckigSmoother: Trajectory complete");
    } else if (result == ruckig::Result::Working) {
        point.isComplete = false;
        // Pass output state to input for next cycle
        m_output.pass_to_input(m_input);
    } else {
        // Error - stop
        LOG_WARN("RuckigSmoother: Error result={}", static_cast<int>(result));
        point.isComplete = true;
        m_active = false;
    }

    return point;
}

void RuckigSmoother::cancelToStop(
    const std::array<double, 6>& currentPos,
    const std::array<double, 6>& currentVel)
{
    // Set target to current position with zero velocity -> smooth decel
    std::array<double, 6> zeroVel{};
    setTarget(currentPos, currentVel, currentPos, zeroVel);
    LOG_DEBUG("RuckigSmoother: Cancel to stop initiated");
}

} // namespace jog
} // namespace robot_controller
