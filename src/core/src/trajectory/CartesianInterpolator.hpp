/**
 * @file CartesianInterpolator.hpp
 * @brief Cartesian space interpolation with S-curve velocity profile
 *
 * Combines DoubleSCurveProfile (time-domain velocity shaping) with
 * spatial interpolation (MoveL / MoveC) for smooth robot motion.
 *
 * - MoveL: Linear position + SLERP orientation
 * - MoveC: Circular arc position + SLERP orientation along arc
 *
 * All methods use the S-curve parameter s(t) as the master interpolation
 * variable, ensuring position and orientation are perfectly synchronized.
 */

#pragma once

#include "DoubleSCurveProfile.hpp"
#include "../kinematics/ForwardKinematics.hpp"
#include "../kinematics/MathTypes.hpp"
#include <Eigen/Geometry>

namespace robot_controller {
namespace trajectory {

using namespace kinematics;

// ============================================================================
// Interpolated State (output at each time step)
// ============================================================================

struct CartesianState {
    TCPPose pose;               // Position + Orientation
    Vector3d linearVelocity;    // mm/s
    Vector3d angularVelocity;   // rad/s
    double pathVelocity;        // scalar speed along path (mm/s)
    double pathAcceleration;    // scalar acceleration along path (mm/s²)
    double pathPosition;        // distance along path (mm)
    bool valid = false;
};

// ============================================================================
// MoveL Segment (Linear)
// ============================================================================

/**
 * Linear Cartesian interpolation with 7-segment S-curve velocity.
 *
 * Position: P(t) = P_start + s(t)/L * (P_end - P_start)
 * Orientation: Q(t) = SLERP(Q_start, Q_end, s(t)/L)
 *
 * where s(t) is the S-curve path position and L is total distance.
 */
class MoveLSegment {
public:
    MoveLSegment() = default;

    /**
     * Plan a linear motion segment.
     *
     * @param start     Start pose
     * @param end       End pose
     * @param v_start   Start velocity along path (mm/s, >= 0)
     * @param v_end     End velocity along path (mm/s, >= 0)
     * @param v_max     Maximum path velocity (mm/s)
     * @param a_max     Maximum path acceleration (mm/s²)
     * @param j_max     Maximum path jerk (mm/s³)
     * @return true if planning succeeded
     */
    bool plan(const TCPPose& start, const TCPPose& end,
              double v_start, double v_end,
              double v_max, double a_max, double j_max);

    /**
     * Evaluate at time t.
     *
     * @param t Time (0 to getDuration())
     * @return Interpolated Cartesian state
     */
    CartesianState evaluate(double t) const;

    double getDuration() const { return profile_.getDuration(); }
    double getPathLength() const { return pathLength_; }
    bool isValid() const { return valid_; }

private:
    TCPPose start_, end_;
    DoubleSCurveProfile profile_;
    double pathLength_ = 0;
    Vector3d direction_ = Vector3d::Zero();  // unit direction vector
    bool valid_ = false;
};

// ============================================================================
// MoveC Segment (Circular Arc)
// ============================================================================

/**
 * Circular arc interpolation with 7-segment S-curve velocity.
 *
 * Position: interpolated along the arc defined by 3 points.
 * Orientation: SLERP(Q_start, Q_end, s(t)/L) along arc length.
 */
class MoveCSegment {
public:
    MoveCSegment() = default;

    /**
     * Plan a circular arc motion segment.
     *
     * @param start     Start pose
     * @param via       Via pose (intermediate point on arc)
     * @param end       End pose
     * @param v_start   Start velocity along path (mm/s, >= 0)
     * @param v_end     End velocity along path (mm/s, >= 0)
     * @param v_max     Maximum path velocity (mm/s)
     * @param a_max     Maximum path acceleration (mm/s²)
     * @param j_max     Maximum path jerk (mm/s³)
     * @return true if planning succeeded
     */
    bool plan(const TCPPose& start, const TCPPose& via, const TCPPose& end,
              double v_start, double v_end,
              double v_max, double a_max, double j_max);

    /**
     * Evaluate at time t.
     */
    CartesianState evaluate(double t) const;

    double getDuration() const { return profile_.getDuration(); }
    double getPathLength() const { return arcLength_; }
    double getRadius() const { return radius_; }
    double getSweepAngle() const { return sweepAngle_; }
    bool isValid() const { return valid_; }

private:
    TCPPose start_, end_;
    DoubleSCurveProfile profile_;

    // Arc geometry
    Vector3d center_;
    Vector3d normal_;
    Vector3d startDir_;     // unit vector from center to start point
    double radius_ = 0;
    double sweepAngle_ = 0;
    double arcLength_ = 0;

    bool valid_ = false;

    /**
     * Compute sweep angle from start through via to end.
     * Returns signed angle (positive = CCW when viewed from normal direction).
     */
    double computeSweepAngle(const Vector3d& start, const Vector3d& via,
                             const Vector3d& end, const Vector3d& center,
                             const Vector3d& normal) const;
};

} // namespace trajectory
} // namespace robot_controller
