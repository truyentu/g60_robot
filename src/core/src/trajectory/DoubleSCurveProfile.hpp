/**
 * @file DoubleSCurveProfile.hpp
 * @brief 7-Segment Double S-Curve velocity profile (jerk-limited)
 *
 * Implements the Biagiotti-Melchiorri Algorithm 3.1 for jerk-limited
 * trajectory planning. Handles all degenerate cases.
 *
 * The 7 phases:
 *   T1: Jerk+ (increasing acceleration)
 *   T2: Constant acceleration
 *   T3: Jerk- (decreasing acceleration to zero)
 *   T4: Constant velocity (cruise)
 *   T5: Jerk- (increasing deceleration)
 *   T6: Constant deceleration
 *   T7: Jerk+ (decreasing deceleration to zero)
 *
 * References:
 *   - Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines
 *     and Robots", Springer 2008, Chapter 3
 *   - Ni et al., "Research on Velocity Look-Ahead Algorithm of the 6-DOF Robot"
 */

#pragma once

#include <cmath>
#include <array>

namespace robot_controller {
namespace trajectory {

struct SCurveState {
    double position;
    double velocity;
    double acceleration;
    double jerk;
};

/**
 * 7-Segment Double S-Curve velocity profile.
 *
 * Stateless after plan(). Thread-safe for evaluate() calls.
 * No dynamic allocation.
 */
class DoubleSCurveProfile {
public:
    DoubleSCurveProfile();

    /**
     * Plan a motion profile.
     *
     * @param q_start    Start position
     * @param q_end      End position
     * @param v_start    Start velocity (>= 0)
     * @param v_end      End velocity (>= 0)
     * @param v_max      Maximum velocity (> 0)
     * @param a_max      Maximum acceleration (> 0)
     * @param j_max      Maximum jerk (> 0)
     * @return true if planning succeeded
     */
    bool plan(double q_start, double q_end,
              double v_start, double v_end,
              double v_max, double a_max, double j_max);

    /**
     * Evaluate profile at given time.
     *
     * @param t Time (0 to getDuration())
     * @return {position, velocity, acceleration, jerk} at time t
     */
    SCurveState evaluate(double t) const;

    /**
     * Get total motion duration.
     */
    double getDuration() const { return totalDuration_; }

    /**
     * Get the 7 phase durations.
     */
    const std::array<double, 7>& getPhaseDurations() const { return T_; }

    /**
     * Get the achieved peak velocity (may be < v_max for short distances).
     */
    double getPeakVelocity() const { return vLim_; }

    /**
     * Check if profile is valid (plan() succeeded).
     */
    bool isValid() const { return valid_; }

private:
    // Planning inputs
    double qStart_, qEnd_;
    double vStart_, vEnd_;
    double vMax_, aMax_, jMax_;
    int sign_;  // +1 or -1 for motion direction

    // Phase durations [T1..T7]
    std::array<double, 7> T_;

    // Cumulative phase end times
    std::array<double, 8> tEnd_;  // tEnd_[0]=0, tEnd_[i]=sum(T_[0..i-1])

    // State at start of each phase [0..7], where [0] = initial state
    std::array<SCurveState, 8> phaseStart_;

    // Achieved peak velocity
    double vLim_;

    // Jerk values for each phase
    std::array<double, 7> jerk_;

    // Acceleration values for each phase (constant part)
    std::array<double, 7> accelConst_;

    double totalDuration_;
    bool valid_;

    // Internal planning methods
    bool computePhaseDurations(double h, double vs, double ve,
                               double vm, double am, double jm);
    void computePhaseStartStates(double q0, double vs);
    void computeCumulativeTimes();

    // Evaluate within a specific phase
    SCurveState evaluatePhase(int phase, double tau) const;
};

} // namespace trajectory
} // namespace robot_controller
