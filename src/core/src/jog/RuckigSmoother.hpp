#pragma once

#include <ruckig/ruckig.hpp>
#include <array>

namespace robot_controller {
namespace jog {

/**
 * Wraps Ruckig OTG for 6-DOF jog trajectory smoothing.
 * Provides S-curve (jerk-limited) velocity profiles.
 */
class RuckigSmoother {
public:
    explicit RuckigSmoother(double cycleTimeSeconds = 0.001);
    ~RuckigSmoother() = default;

    void setJointLimits(int joint, double maxVel, double maxAccel, double maxJerk);

    void setAllLimits(
        const std::array<double, 6>& maxVelocities,
        const std::array<double, 6>& maxAccelerations,
        const std::array<double, 6>& maxJerks);

    bool setTarget(
        const std::array<double, 6>& currentPos,
        const std::array<double, 6>& currentVel,
        const std::array<double, 6>& targetPos,
        const std::array<double, 6>& targetVel = {0,0,0,0,0,0});

    struct TrajectoryPoint {
        std::array<double, 6> position;
        std::array<double, 6> velocity;
        std::array<double, 6> acceleration;
        bool isComplete;
    };

    TrajectoryPoint update();

    void cancelToStop(const std::array<double, 6>& currentPos,
                      const std::array<double, 6>& currentVel);

    bool isActive() const { return m_active; }

private:
    ruckig::Ruckig<6> m_otg;
    ruckig::InputParameter<6> m_input;
    ruckig::OutputParameter<6> m_output;
    bool m_active{false};
};

} // namespace jog
} // namespace robot_controller
