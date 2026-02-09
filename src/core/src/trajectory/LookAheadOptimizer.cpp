/**
 * @file LookAheadOptimizer.cpp
 * @brief Implementation of adaptive velocity look-ahead
 *
 * The algorithm runs in 4 passes:
 *   Pass 1: Compute corner velocities at each junction
 *   Pass 2: Limit cruise velocity by segment length
 *   Pass 3: Backward pass — deceleration feasibility
 *   Pass 4: Forward pass — acceleration feasibility
 */

#include "LookAheadOptimizer.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_controller {
namespace trajectory {

// ============================================================================
// Corner Angle
// ============================================================================

double LookAheadOptimizer::cornerAngle(
    double dx1, double dy1, double dz1,
    double dx2, double dy2, double dz2)
{
    // Dot product of direction vectors
    double dot = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
    dot = std::clamp(dot, -1.0, 1.0);

    // Angle between directions (0 = same direction, π = reversal)
    return std::acos(dot);
}

// ============================================================================
// Max Corner Velocity
// ============================================================================

double LookAheadOptimizer::maxCornerVelocity(
    double angle, double a_max, double j_max, double deviation)
{
    // For very small angles (nearly straight), allow full speed
    if (angle < 1e-6) {
        return std::numeric_limits<double>::max();
    }

    // For reversal (angle ≈ π), must stop
    if (angle > M_PI - 1e-6) {
        return 0;
    }

    double half_angle = angle / 2.0;
    double sin_half = std::sin(half_angle);

    if (sin_half < 1e-10) {
        return std::numeric_limits<double>::max();
    }

    // Method 1: Based on deviation tolerance
    // At a corner with velocity v, the deviation from the ideal path is:
    //   δ = v² * sin(θ/2) / (2 * a_max * cos(θ/2))
    // Solving for v:
    //   v = sqrt(2 * a_max * deviation * cos(θ/2) / sin(θ/2))
    double cos_half = std::cos(half_angle);
    double v_dev = std::sqrt(2.0 * a_max * deviation * cos_half / sin_half);

    // Method 2: Based on jerk constraint (from paper)
    // v_corner = a_max * Tj / (2 * sin(θ/2))
    // where Tj = a_max / j_max (jerk transition time)
    double Tj = a_max / j_max;
    double v_jerk = a_max * Tj / (2.0 * sin_half);

    // Use the more conservative (smaller) limit
    return std::min(v_dev, v_jerk);
}

// ============================================================================
// Max Reachable Velocity
// ============================================================================

double LookAheadOptimizer::maxReachableVelocity(
    double v_start, double distance, double a_max, double j_max)
{
    if (distance <= 0) return v_start;

    // For S-curve acceleration from v_start over distance d:
    // Using the simplified formula from the paper:
    //   S_accel = (v² - v_s²) / (2*a) + a*v/(2*j) + a*v_s/(2*j)
    // This is complex to invert. Use a conservative estimate:
    //
    // For trapezoidal approximation (upper bound):
    //   v_max = sqrt(v_start² + 2*a_max*distance)
    //
    // For S-curve (jerk-limited), the actual reachable velocity is lower.
    // Account for jerk by reducing effective acceleration:
    //   The jerk phase uses distance d_jerk = a_max² / (2*j_max) * v_avg
    //   Effective distance for const-accel phase = distance - overhead

    double Tj = a_max / j_max;

    // Distance consumed by jerk ramp-up and ramp-down
    // During jerk phase: v changes by 0.5 * j_max * Tj² = 0.5 * a_max² / j_max
    // Distance during jerk phase ≈ v_start * Tj + a_max * Tj² / 6
    double d_jerk = v_start * Tj + a_max * Tj * Tj / 6.0;

    // If we need two jerk phases (ramp up and ramp down acceleration)
    double d_jerk_total = 2.0 * d_jerk;

    if (distance <= d_jerk_total) {
        // Very short distance — triangular jerk profile
        // v_max ≈ v_start + j_max * (d / (2*v_start + epsilon))
        // Simplified: use sqrt approximation
        double v_tri = std::sqrt(v_start * v_start + 2.0 * a_max * distance * 0.75);
        return v_tri;
    }

    // Remaining distance for constant acceleration
    double d_const = distance - d_jerk_total;
    double v_after_jerk = v_start + 0.5 * a_max * a_max / j_max;

    // v² = v_after_jerk² + 2 * a_max * d_const
    double v_sq = v_after_jerk * v_after_jerk + 2.0 * a_max * d_const;
    if (v_sq < 0) return v_start;

    return std::sqrt(v_sq);
}

// ============================================================================
// Transition Distance (S-curve)
// ============================================================================

double LookAheadOptimizer::transitionDistance(
    double v1, double v2, double a_max, double j_max)
{
    if (std::abs(v2 - v1) < 1e-10) return 0;

    double v_high = std::max(v1, v2);
    double v_low = std::min(v1, v2);
    double dv = v_high - v_low;

    double Tj = a_max / j_max;

    // Check if a_max is reachable
    if (dv < a_max * a_max / j_max) {
        // Triangular acceleration profile (T2 = 0)
        // Tj_actual = sqrt(dv / j_max)
        double Tj_act = std::sqrt(dv / j_max);
        double Ta = 2.0 * Tj_act;
        // Distance = (v_high + v_low) / 2 * Ta
        return (v_high + v_low) / 2.0 * Ta;
    }

    // Trapezoidal acceleration profile
    // Ta = Tj + dv / a_max
    double Ta = Tj + dv / a_max;

    // Distance = (v_high + v_low) / 2 * Ta
    return (v_high + v_low) / 2.0 * Ta;
}

// ============================================================================
// Main Optimize
// ============================================================================

void LookAheadOptimizer::optimize(
    std::vector<PathSegment>& segments,
    double v_entry, double v_exit)
{
    int N = static_cast<int>(segments.size());
    if (N == 0) return;

    // ========================================================================
    // Pass 1: Compute corner velocities at each junction
    // ========================================================================

    // Junction velocities: v_junction[i] = max velocity at junction between
    // segment i-1 and segment i. v_junction[0] = v_entry, v_junction[N] = v_exit.
    std::vector<double> v_junction(N + 1);
    v_junction[0] = v_entry;
    v_junction[N] = v_exit;

    for (int i = 1; i < N; ++i) {
        double angle = cornerAngle(
            segments[i - 1].dx, segments[i - 1].dy, segments[i - 1].dz,
            segments[i].dx, segments[i].dy, segments[i].dz);

        double v_corner = maxCornerVelocity(
            angle,
            std::min(segments[i - 1].a_max, segments[i].a_max),
            std::min(segments[i - 1].j_max, segments[i].j_max));

        // Also limit by segment v_max on both sides
        v_corner = std::min(v_corner, segments[i - 1].v_max);
        v_corner = std::min(v_corner, segments[i].v_max);

        v_junction[i] = v_corner;
    }

    // ========================================================================
    // Pass 2: Limit junction velocities by segment length
    // ========================================================================

    // Each segment constrains the velocity at its start and end:
    // the segment must have enough distance to transition between them.
    for (int i = 0; i < N; ++i) {
        double v_max_seg = maxReachableVelocity(
            0, segments[i].length / 2.0,
            segments[i].a_max, segments[i].j_max);

        // Segment can't sustain velocity higher than what it can reach
        // from zero in half its length
        v_max_seg = std::min(v_max_seg, segments[i].v_max);
        v_junction[i] = std::min(v_junction[i], v_max_seg);
        v_junction[i + 1] = std::min(v_junction[i + 1], v_max_seg);
    }

    // ========================================================================
    // Pass 3: Backward pass — deceleration feasibility
    // ========================================================================

    // Starting from the last junction, propagate backward.
    // If segment i can't decelerate from v_junction[i] to v_junction[i+1],
    // then v_junction[i] must be reduced.
    for (int i = N - 1; i >= 0; --i) {
        double v_end = v_junction[i + 1];
        double v_reachable = maxReachableVelocity(
            v_end, segments[i].length,
            segments[i].a_max, segments[i].j_max);

        v_reachable = std::min(v_reachable, segments[i].v_max);
        v_junction[i] = std::min(v_junction[i], v_reachable);
    }

    // ========================================================================
    // Pass 4: Forward pass — acceleration feasibility
    // ========================================================================

    // Starting from the first junction, propagate forward.
    // If segment i can't accelerate from v_junction[i] to v_junction[i+1],
    // then v_junction[i+1] must be reduced.
    for (int i = 0; i < N; ++i) {
        double v_start = v_junction[i];
        double v_reachable = maxReachableVelocity(
            v_start, segments[i].length,
            segments[i].a_max, segments[i].j_max);

        v_reachable = std::min(v_reachable, segments[i].v_max);
        v_junction[i + 1] = std::min(v_junction[i + 1], v_reachable);
    }

    // ========================================================================
    // Assign results
    // ========================================================================

    for (int i = 0; i < N; ++i) {
        segments[i].v_start = v_junction[i];
        segments[i].v_end = v_junction[i + 1];

        // Cruise velocity: limited by v_max and what's reachable from both ends
        double v_from_start = maxReachableVelocity(
            segments[i].v_start, segments[i].length / 2.0,
            segments[i].a_max, segments[i].j_max);
        double v_from_end = maxReachableVelocity(
            segments[i].v_end, segments[i].length / 2.0,
            segments[i].a_max, segments[i].j_max);

        segments[i].v_cruise = std::min({
            segments[i].v_max,
            v_from_start,
            v_from_end
        });
    }
}

} // namespace trajectory
} // namespace robot_controller
