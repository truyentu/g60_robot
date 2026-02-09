/**
 * @file CartesianInterpolator.cpp
 * @brief Implementation of MoveL and MoveC Cartesian interpolation
 *
 * Both use DoubleSCurveProfile as the master timing source.
 * The S-curve parameter s(t) maps time → path distance,
 * which then maps to spatial position via geometry.
 */

#include "CartesianInterpolator.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_controller {
namespace trajectory {

// ============================================================================
// MoveLSegment — Linear Interpolation
// ============================================================================

bool MoveLSegment::plan(
    const TCPPose& start, const TCPPose& end,
    double v_start, double v_end,
    double v_max, double a_max, double j_max)
{
    valid_ = false;
    start_ = start;
    end_ = end;

    // Compute path length (position distance)
    Vector3d delta = end.position - start.position;
    pathLength_ = delta.norm();

    if (pathLength_ < 1e-6) {
        // Zero distance — orientation-only move
        // Use angular distance as path length proxy
        double dot = std::abs(start.quaternion.dot(end.quaternion));
        dot = std::min(dot, 1.0);
        double angularDist = 2.0 * std::acos(dot);

        if (angularDist < 1e-8) {
            // No motion at all
            pathLength_ = 0;
            direction_ = Vector3d::Zero();
            valid_ = true;
            return true;
        }

        // Use 1mm per degree as proxy scaling
        pathLength_ = angularDist * (180.0 / M_PI);
        direction_ = Vector3d::Zero();
    } else {
        direction_ = delta / pathLength_;
    }

    // Plan S-curve velocity profile along path
    if (!profile_.plan(0, pathLength_, v_start, v_end, v_max, a_max, j_max)) {
        return false;
    }

    valid_ = true;
    return true;
}

CartesianState MoveLSegment::evaluate(double t) const {
    CartesianState state;
    state.valid = false;

    if (!valid_) return state;

    // Handle zero-length motion
    if (pathLength_ < 1e-6) {
        state.pose = start_;
        state.linearVelocity = Vector3d::Zero();
        state.angularVelocity = Vector3d::Zero();
        state.pathVelocity = 0;
        state.pathAcceleration = 0;
        state.pathPosition = 0;
        state.valid = true;
        return state;
    }

    // Evaluate S-curve at time t
    auto sc = profile_.evaluate(t);
    double s = sc.position;  // path distance traveled

    // Normalized parameter (0..1)
    double u = std::clamp(s / pathLength_, 0.0, 1.0);

    // Position: linear interpolation
    state.pose.position = start_.position + u * (end_.position - start_.position);

    // Orientation: SLERP
    state.pose.quaternion = start_.quaternion.slerp(u, end_.quaternion);
    state.pose.quaternion.normalize();
    state.pose.rotation = state.pose.quaternion.toRotationMatrix();
    state.pose.rpy = rotationToRPY(state.pose.rotation);

    // Velocities
    state.pathVelocity = sc.velocity;
    state.pathAcceleration = sc.acceleration;
    state.pathPosition = s;

    // Linear velocity = direction * path_speed
    state.linearVelocity = direction_ * sc.velocity;

    // Angular velocity (finite difference approximation)
    // For SLERP: ω = 2 * (dq/dt) * q^(-1), simplified:
    if (pathLength_ > 1e-6 && std::abs(sc.velocity) > 1e-10) {
        double du_dt = sc.velocity / pathLength_;  // du/dt
        double dt_small = 1e-6;
        double u2 = std::clamp((s + sc.velocity * dt_small) / pathLength_, 0.0, 1.0);

        Quaterniond q1 = state.pose.quaternion;
        Quaterniond q2 = start_.quaternion.slerp(u2, end_.quaternion);
        q2.normalize();

        // ω ≈ 2 * (q2 - q1) / dt * q1^(-1)
        // Simplified: use the axis-angle between q1 and q2
        Quaterniond dq = q1.conjugate() * q2;
        if (dq.w() < 0) {
            dq.coeffs() = -dq.coeffs();
        }
        Eigen::AngleAxisd aa(dq);
        state.angularVelocity = aa.axis() * aa.angle() / dt_small;
    } else {
        state.angularVelocity = Vector3d::Zero();
    }

    state.valid = true;
    return state;
}

// ============================================================================
// MoveCSegment — Circular Arc Interpolation
// ============================================================================

bool MoveCSegment::plan(
    const TCPPose& start, const TCPPose& via, const TCPPose& end,
    double v_start, double v_end,
    double v_max, double a_max, double j_max)
{
    valid_ = false;
    start_ = start;
    end_ = end;

    // ---- Step 1: Compute arc geometry from 3 points ----
    Vector3d p1 = start.position;
    Vector3d p2 = via.position;
    Vector3d p3 = end.position;

    // Vectors
    Vector3d v1 = p2 - p1;
    Vector3d v2 = p3 - p1;

    // Normal to arc plane
    normal_ = v1.cross(v2);
    double normalLen = normal_.norm();

    if (normalLen < 1e-6) {
        // Points are collinear — fall back to linear
        return false;
    }
    normal_.normalize();

    // Find circumcenter using perpendicular bisector method
    // Midpoints
    Vector3d mid1 = 0.5 * (p1 + p2);
    Vector3d mid2 = 0.5 * (p2 + p3);

    // Perpendicular bisector directions (in the arc plane)
    Vector3d d1 = normal_.cross(v1);  // perp to v1=(p2-p1), in plane
    Vector3d d2 = normal_.cross(p3 - p2);  // perp to (p3-p2), in plane

    // Find intersection: mid1 + t*d1 = mid2 + s*d2
    // Solve for t using least squares
    // (mid1 + t*d1 - mid2) × d2 = 0
    // t * (d1 × d2) = (mid2 - mid1) × d2
    Vector3d cross_d1d2 = d1.cross(d2);
    double denom = cross_d1d2.squaredNorm();

    if (denom < 1e-12) return false;

    Vector3d diff = mid2 - mid1;
    double t_param = diff.cross(d2).dot(cross_d1d2) / denom;

    center_ = mid1 + t_param * d1;
    radius_ = (p1 - center_).norm();

    // ---- Step 2: Compute start direction and sweep angle ----
    startDir_ = (p1 - center_).normalized();

    sweepAngle_ = computeSweepAngle(p1, p2, p3, center_, normal_);
    arcLength_ = std::abs(radius_ * sweepAngle_);

    if (arcLength_ < 1e-6) return false;

    // ---- Step 3: Plan S-curve along arc length ----
    if (!profile_.plan(0, arcLength_, v_start, v_end, v_max, a_max, j_max)) {
        return false;
    }

    valid_ = true;
    return true;
}

double MoveCSegment::computeSweepAngle(
    const Vector3d& start, const Vector3d& via,
    const Vector3d& end, const Vector3d& center,
    const Vector3d& normal) const
{
    Vector3d r_start = (start - center).normalized();
    Vector3d r_via = (via - center).normalized();
    Vector3d r_end = (end - center).normalized();

    // Angle from start to via
    double cos_sv = std::clamp(r_start.dot(r_via), -1.0, 1.0);
    double angle_sv = std::acos(cos_sv);
    // Check direction using cross product
    if (r_start.cross(r_via).dot(normal) < 0) {
        angle_sv = -angle_sv;
    }

    // Angle from start to end
    double cos_se = std::clamp(r_start.dot(r_end), -1.0, 1.0);
    double angle_se = std::acos(cos_se);
    if (r_start.cross(r_end).dot(normal) < 0) {
        angle_se = -angle_se;
    }

    // Ensure via is between start and end
    // If angle_sv and angle_se have same sign, and |angle_sv| < |angle_se|,
    // then arc goes start → via → end in the shorter direction.
    // If not, we need the longer arc (2π - angle).
    if (angle_sv * angle_se > 0 && std::abs(angle_sv) < std::abs(angle_se)) {
        return angle_se;
    } else if (angle_sv * angle_se > 0) {
        // Via is past end — go the long way
        if (angle_se > 0) {
            return angle_se - 2.0 * M_PI;
        } else {
            return angle_se + 2.0 * M_PI;
        }
    } else {
        // Different signs: via on opposite side
        if (angle_se > 0) {
            return angle_se - 2.0 * M_PI;
        } else {
            return angle_se + 2.0 * M_PI;
        }
    }
}

CartesianState MoveCSegment::evaluate(double t) const {
    CartesianState state;
    state.valid = false;

    if (!valid_) return state;

    // Evaluate S-curve at time t
    auto sc = profile_.evaluate(t);
    double s = sc.position;  // arc distance traveled

    // Normalized parameter (0..1)
    double u = std::clamp(s / arcLength_, 0.0, 1.0);

    // ---- Position: rotate around arc ----
    double angle = u * sweepAngle_;
    Eigen::AngleAxisd rot(angle, normal_);
    Vector3d dir = rot * startDir_;
    state.pose.position = center_ + radius_ * dir;

    // ---- Orientation: SLERP between start and end ----
    state.pose.quaternion = start_.quaternion.slerp(u, end_.quaternion);
    state.pose.quaternion.normalize();
    state.pose.rotation = state.pose.quaternion.toRotationMatrix();
    state.pose.rpy = rotationToRPY(state.pose.rotation);

    // ---- Velocities ----
    state.pathVelocity = sc.velocity;
    state.pathAcceleration = sc.acceleration;
    state.pathPosition = s;

    // Linear velocity: tangent to arc * speed
    // Tangent = normal × radial direction
    Vector3d tangent = normal_.cross(dir);
    if (sweepAngle_ < 0) tangent = -tangent;
    state.linearVelocity = tangent * sc.velocity;

    // Angular velocity (same as MoveL — finite difference)
    state.angularVelocity = Vector3d::Zero();  // simplified for now

    state.valid = true;
    return state;
}

} // namespace trajectory
} // namespace robot_controller
