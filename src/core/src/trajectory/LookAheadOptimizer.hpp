/**
 * @file LookAheadOptimizer.hpp
 * @brief Adaptive velocity look-ahead for continuous path motion
 *
 * Processes a queue of motion segments and assigns optimal start/end
 * velocities so the robot maintains maximum speed on straight paths
 * and smoothly decelerates at corners — without stopping.
 *
 * Algorithm (based on "Research on Velocity Look-Ahead Algorithm of the
 * 6-DOF Robot", Ni et al., 2015):
 *
 *   1. Corner velocity: compute max junction speed from turn angle
 *   2. Segment velocity: limit by segment length (can't exceed what
 *      the S-curve can accelerate/decelerate within the distance)
 *   3. Backward pass: propagate deceleration constraints from end
 *   4. Forward pass: propagate acceleration constraints from start
 *   5. Output: optimal v_start, v_end for each segment
 *
 * References:
 *   - Ni, X.; Liu, J.; Gao, W. "Research on Velocity Look-Ahead
 *     Algorithm of the 6-DOF Robot." ICSEEE 2015.
 *   - Biagiotti, L.; Melchiorri, C. "Trajectory Planning for Automatic
 *     Machines and Robots." Springer 2008.
 */

#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace robot_controller {
namespace trajectory {

// ============================================================================
// Motion Segment Description (input to optimizer)
// ============================================================================

struct PathSegment {
    double length;        // Segment length (mm)

    // Direction unit vector (for corner angle computation)
    double dx, dy, dz;    // Normalized direction

    // Constraints
    double v_max;         // Max velocity for this segment (mm/s)
    double a_max;         // Max acceleration (mm/s²)
    double j_max;         // Max jerk (mm/s³)

    // Output (filled by optimizer)
    double v_start;       // Optimized start velocity (mm/s)
    double v_end;         // Optimized end velocity (mm/s)
    double v_cruise;      // Achievable cruise velocity (mm/s)
};

// ============================================================================
// Look-Ahead Optimizer
// ============================================================================

class LookAheadOptimizer {
public:
    LookAheadOptimizer() = default;

    /**
     * Optimize velocities for a sequence of path segments.
     *
     * After calling this, each segment's v_start, v_end, v_cruise
     * fields are filled with optimized values.
     *
     * @param segments  Vector of path segments (modified in place)
     * @param v_entry   Entry velocity of first segment (typically 0)
     * @param v_exit    Exit velocity of last segment (typically 0)
     */
    void optimize(std::vector<PathSegment>& segments,
                  double v_entry = 0, double v_exit = 0);

    /**
     * Compute corner angle between two direction vectors.
     *
     * @return Angle in radians (0 = collinear, π = reversal)
     */
    static double cornerAngle(double dx1, double dy1, double dz1,
                              double dx2, double dy2, double dz2);

    /**
     * Compute maximum junction velocity for a given corner angle.
     *
     * Based on paper: v_corner = (a_max * T) / (2 * sin(θ/2))
     * where T is the jerk transition time.
     *
     * @param angle       Corner angle in radians
     * @param a_max       Maximum acceleration (mm/s²)
     * @param j_max       Maximum jerk (mm/s³)
     * @param deviation   Maximum allowed path deviation (mm, default 0.5)
     * @return Maximum corner velocity (mm/s)
     */
    static double maxCornerVelocity(double angle, double a_max,
                                    double j_max, double deviation = 0.5);

    /**
     * Compute maximum reachable velocity over a given distance
     * starting from v_start, using S-curve constraints.
     *
     * @param v_start   Start velocity (mm/s)
     * @param distance  Available distance (mm)
     * @param a_max     Maximum acceleration (mm/s²)
     * @param j_max     Maximum jerk (mm/s³)
     * @return Maximum velocity achievable
     */
    static double maxReachableVelocity(double v_start, double distance,
                                       double a_max, double j_max);

    /**
     * Compute minimum distance needed to change from v1 to v2
     * using S-curve profile.
     *
     * @return Distance required (mm)
     */
    static double transitionDistance(double v1, double v2,
                                    double a_max, double j_max);
};

} // namespace trajectory
} // namespace robot_controller
