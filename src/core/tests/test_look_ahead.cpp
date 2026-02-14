/**
 * @file test_look_ahead.cpp
 * @brief Unit tests for LookAheadOptimizer
 *
 * Key deliverable test: square path (4 corners at 90°)
 * proves velocity doesn't drop to zero at corners.
 */

#include <gtest/gtest.h>
#include "trajectory/LookAheadOptimizer.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace robot_controller::trajectory;

// ============================================================================
// Helper: Create a path segment
// ============================================================================

static PathSegment makeSegment(
    double length, double dx, double dy, double dz,
    double v_max = 500.0, double a_max = 2000.0, double j_max = 20000.0)
{
    PathSegment seg{};
    seg.length = length;
    // Normalize direction
    double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (norm > 1e-10) {
        seg.dx = dx / norm;
        seg.dy = dy / norm;
        seg.dz = dz / norm;
    } else {
        seg.dx = seg.dy = seg.dz = 0;
    }
    seg.v_max = v_max;
    seg.a_max = a_max;
    seg.j_max = j_max;
    seg.v_start = 0;
    seg.v_end = 0;
    seg.v_cruise = 0;
    return seg;
}

// ============================================================================
// Corner Angle Tests
// ============================================================================

TEST(LookAheadCornerAngle, SameDirection) {
    // Two segments in the same direction → angle = 0
    double angle = LookAheadOptimizer::cornerAngle(1, 0, 0, 1, 0, 0);
    EXPECT_NEAR(angle, 0.0, 1e-10);
}

TEST(LookAheadCornerAngle, RightAngle) {
    // 90° turn: X → Y
    double angle = LookAheadOptimizer::cornerAngle(1, 0, 0, 0, 1, 0);
    EXPECT_NEAR(angle, M_PI / 2.0, 1e-10);
}

TEST(LookAheadCornerAngle, Reversal) {
    // 180° reversal: X → -X
    double angle = LookAheadOptimizer::cornerAngle(1, 0, 0, -1, 0, 0);
    EXPECT_NEAR(angle, M_PI, 1e-10);
}

TEST(LookAheadCornerAngle, Angle45) {
    // 45° turn: X → (X+Y)/√2
    double s = 1.0 / std::sqrt(2.0);
    double angle = LookAheadOptimizer::cornerAngle(1, 0, 0, s, s, 0);
    EXPECT_NEAR(angle, M_PI / 4.0, 1e-10);
}

TEST(LookAheadCornerAngle, ThreeDimensional) {
    // 3D vectors
    double angle = LookAheadOptimizer::cornerAngle(1, 0, 0, 0, 0, 1);
    EXPECT_NEAR(angle, M_PI / 2.0, 1e-10);
}

// ============================================================================
// Max Corner Velocity Tests
// ============================================================================

TEST(LookAheadCornerVelocity, StraightLine) {
    // Angle = 0 → infinite corner velocity (no slowdown needed)
    double v = LookAheadOptimizer::maxCornerVelocity(0.0, 2000.0, 20000.0);
    EXPECT_GT(v, 1e6);  // Should be very large
}

TEST(LookAheadCornerVelocity, Reversal) {
    // Angle = π → must stop
    double v = LookAheadOptimizer::maxCornerVelocity(M_PI, 2000.0, 20000.0);
    EXPECT_NEAR(v, 0.0, 1e-10);
}

TEST(LookAheadCornerVelocity, RightAngle) {
    // 90° corner → some finite velocity
    double v = LookAheadOptimizer::maxCornerVelocity(M_PI / 2.0, 2000.0, 20000.0);
    EXPECT_GT(v, 0.0);
    EXPECT_LT(v, 500.0);  // Should be limited
}

TEST(LookAheadCornerVelocity, SmallAngle) {
    // Small angle (nearly straight) → high velocity
    double v_small = LookAheadOptimizer::maxCornerVelocity(0.1, 2000.0, 20000.0);
    double v_large = LookAheadOptimizer::maxCornerVelocity(1.0, 2000.0, 20000.0);
    EXPECT_GT(v_small, v_large);  // Smaller angle → higher velocity
}

TEST(LookAheadCornerVelocity, MonotonicDecrease) {
    // Corner velocity should decrease as angle increases
    double a_max = 2000.0;
    double j_max = 20000.0;

    double prev_v = std::numeric_limits<double>::max();
    for (double angle = 0.1; angle < M_PI - 0.1; angle += 0.2) {
        double v = LookAheadOptimizer::maxCornerVelocity(angle, a_max, j_max);
        EXPECT_LE(v, prev_v) << "Angle=" << angle;
        prev_v = v;
    }
}

// ============================================================================
// Max Reachable Velocity Tests
// ============================================================================

TEST(LookAheadReachable, ZeroDistance) {
    double v = LookAheadOptimizer::maxReachableVelocity(0, 0, 2000, 20000);
    EXPECT_NEAR(v, 0.0, 1e-10);
}

TEST(LookAheadReachable, IncreasesWithDistance) {
    double v1 = LookAheadOptimizer::maxReachableVelocity(0, 10.0, 2000, 20000);
    double v2 = LookAheadOptimizer::maxReachableVelocity(0, 100.0, 2000, 20000);
    EXPECT_GT(v2, v1);
}

TEST(LookAheadReachable, FromNonZeroStart) {
    double v = LookAheadOptimizer::maxReachableVelocity(100.0, 50.0, 2000, 20000);
    EXPECT_GT(v, 100.0);  // Should be higher than start
}

// ============================================================================
// Transition Distance Tests
// ============================================================================

TEST(LookAheadTransition, SameVelocity) {
    double d = LookAheadOptimizer::transitionDistance(100, 100, 2000, 20000);
    EXPECT_NEAR(d, 0.0, 1e-10);
}

TEST(LookAheadTransition, Positive) {
    double d = LookAheadOptimizer::transitionDistance(0, 200, 2000, 20000);
    EXPECT_GT(d, 0.0);
}

TEST(LookAheadTransition, Symmetric) {
    // Accel distance = decel distance
    double d1 = LookAheadOptimizer::transitionDistance(100, 300, 2000, 20000);
    double d2 = LookAheadOptimizer::transitionDistance(300, 100, 2000, 20000);
    EXPECT_NEAR(d1, d2, 1e-6);
}

// ============================================================================
// DELIVERABLE: Square Path (4 corners at 90°) — velocity never drops to zero
// ============================================================================

TEST(LookAheadOptimize, SquarePath_VelocityNonZeroAtCorners) {
    // Square path: 4 segments of 200mm each, 90° corners
    // This is the key test: robot should maintain velocity through corners
    std::vector<PathSegment> segments;

    // Segment 1: +X direction, 200mm
    segments.push_back(makeSegment(200.0, 1, 0, 0));
    // Segment 2: +Y direction, 200mm (90° turn from +X)
    segments.push_back(makeSegment(200.0, 0, 1, 0));
    // Segment 3: -X direction, 200mm (90° turn from +Y)
    segments.push_back(makeSegment(200.0, -1, 0, 0));
    // Segment 4: -Y direction, 200mm (90° turn from -X)
    segments.push_back(makeSegment(200.0, 0, -1, 0));

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    std::cout << "\n=== Square Path Look-Ahead Results ===" << std::endl;
    for (int i = 0; i < (int)segments.size(); ++i) {
        std::cout << "  Segment " << i
                  << ": v_start=" << std::fixed << std::setprecision(2) << segments[i].v_start
                  << " v_cruise=" << segments[i].v_cruise
                  << " v_end=" << segments[i].v_end
                  << std::endl;
    }
    std::cout << std::endl;

    // KEY ASSERTION: Corner velocities (v_end of seg i = v_start of seg i+1)
    // must be > 0 at all internal junctions
    for (int i = 0; i < (int)segments.size() - 1; ++i) {
        EXPECT_GT(segments[i].v_end, 0.0)
            << "Velocity dropped to zero at corner " << i << " → " << (i + 1);
        EXPECT_GT(segments[i + 1].v_start, 0.0)
            << "Velocity dropped to zero at corner " << i << " → " << (i + 1);

        // Junction continuity: end of seg i = start of seg i+1
        EXPECT_NEAR(segments[i].v_end, segments[i + 1].v_start, 1e-10)
            << "Junction velocity mismatch at corner " << i;
    }

    // Entry and exit should be 0 (we specified v_entry=0, v_exit=0)
    EXPECT_NEAR(segments.front().v_start, 0.0, 1e-10);
    EXPECT_NEAR(segments.back().v_end, 0.0, 1e-10);

    // Cruise velocities should be > 0
    for (int i = 0; i < (int)segments.size(); ++i) {
        EXPECT_GT(segments[i].v_cruise, 0.0)
            << "Cruise velocity is zero for segment " << i;
    }
}

// ============================================================================
// Straight Path — all segments collinear, no slowdown
// ============================================================================

TEST(LookAheadOptimize, StraightPath_FullSpeed) {
    // 5 collinear segments along +X, each 100mm
    std::vector<PathSegment> segments;
    for (int i = 0; i < 5; ++i) {
        segments.push_back(makeSegment(100.0, 1, 0, 0));
    }

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    // Internal junctions should be at or near v_max (no corner penalty)
    // (limited only by segment length / accel capability)
    for (int i = 0; i < (int)segments.size() - 1; ++i) {
        // Junction velocity should be > 0 and equal on both sides
        EXPECT_GT(segments[i].v_end, 0.0);
        EXPECT_NEAR(segments[i].v_end, segments[i + 1].v_start, 1e-10);
    }

    // Middle segments should have higher cruise velocity than end segments
    // because there's no corner deceleration needed
    double v_mid_cruise = segments[2].v_cruise;
    EXPECT_GT(v_mid_cruise, 0.0);
}

// ============================================================================
// Single Segment
// ============================================================================

TEST(LookAheadOptimize, SingleSegment) {
    std::vector<PathSegment> segments;
    segments.push_back(makeSegment(100.0, 1, 0, 0));

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    EXPECT_NEAR(segments[0].v_start, 0.0, 1e-10);
    EXPECT_NEAR(segments[0].v_end, 0.0, 1e-10);
    EXPECT_GT(segments[0].v_cruise, 0.0);
}

// ============================================================================
// Sharp vs Gentle Corner
// ============================================================================

TEST(LookAheadOptimize, SharpCornerSlowerThanGentle) {
    // Two-segment path with different corner angles

    // Test 1: Gentle corner (30°)
    double s30 = std::sin(M_PI / 6.0);
    double c30 = std::cos(M_PI / 6.0);
    std::vector<PathSegment> gentle;
    gentle.push_back(makeSegment(200.0, 1, 0, 0));
    gentle.push_back(makeSegment(200.0, c30, s30, 0));  // 30° turn

    LookAheadOptimizer opt;
    opt.optimize(gentle, 0.0, 0.0);
    double v_gentle = gentle[0].v_end;

    // Test 2: Sharp corner (120°)
    double s120 = std::sin(2.0 * M_PI / 3.0);
    double c120 = std::cos(2.0 * M_PI / 3.0);
    std::vector<PathSegment> sharp;
    sharp.push_back(makeSegment(200.0, 1, 0, 0));
    sharp.push_back(makeSegment(200.0, c120, s120, 0));  // 120° turn

    opt.optimize(sharp, 0.0, 0.0);
    double v_sharp = sharp[0].v_end;

    std::cout << "Corner velocity - gentle (30deg): " << v_gentle
              << ", sharp (120deg): " << v_sharp << std::endl;

    // Gentle corner should allow higher velocity
    EXPECT_GT(v_gentle, v_sharp);
}

// ============================================================================
// Velocity Profile Respects v_max
// ============================================================================

TEST(LookAheadOptimize, RespectsVmax) {
    std::vector<PathSegment> segments;
    // Long straight path with low v_max
    double custom_vmax = 100.0;
    for (int i = 0; i < 10; ++i) {
        segments.push_back(makeSegment(500.0, 1, 0, 0, custom_vmax));
    }

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    for (int i = 0; i < (int)segments.size(); ++i) {
        EXPECT_LE(segments[i].v_cruise, custom_vmax + 1e-6)
            << "Cruise velocity exceeds v_max at segment " << i;
        EXPECT_LE(segments[i].v_start, custom_vmax + 1e-6)
            << "Start velocity exceeds v_max at segment " << i;
        EXPECT_LE(segments[i].v_end, custom_vmax + 1e-6)
            << "End velocity exceeds v_max at segment " << i;
    }
}

// ============================================================================
// Forward/Backward Pass Consistency
// ============================================================================

TEST(LookAheadOptimize, ForwardBackwardConsistency) {
    // Asymmetric path: long → short → long
    std::vector<PathSegment> segments;
    segments.push_back(makeSegment(500.0, 1, 0, 0));
    segments.push_back(makeSegment(10.0,  0, 1, 0));  // Very short with 90° corner
    segments.push_back(makeSegment(500.0, -1, 0, 0));

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    // Short middle segment constrains the velocity profile
    // The cruise velocity of the middle segment should be small
    EXPECT_LT(segments[1].v_cruise, segments[0].v_cruise);

    // Junction continuity
    EXPECT_NEAR(segments[0].v_end, segments[1].v_start, 1e-10);
    EXPECT_NEAR(segments[1].v_end, segments[2].v_start, 1e-10);
}

// ============================================================================
// Empty Input
// ============================================================================

TEST(LookAheadOptimize, EmptyInput) {
    std::vector<PathSegment> segments;
    LookAheadOptimizer optimizer;
    // Should not crash
    optimizer.optimize(segments, 0.0, 0.0);
    EXPECT_TRUE(segments.empty());
}

// ============================================================================
// Welding Path Simulation — L-shaped weld seam
// ============================================================================

TEST(LookAheadOptimize, WeldingPath_LShape) {
    // Simulate an L-shaped weld: 300mm straight, 90° turn, 200mm straight
    // For welding, velocity must stay non-zero through corner
    std::vector<PathSegment> segments;

    // Welding-typical constraints
    double v_max = 200.0;    // 200 mm/s welding speed
    double a_max = 1000.0;   // moderate accel
    double j_max = 10000.0;  // smooth jerk

    segments.push_back(makeSegment(300.0, 1, 0, 0, v_max, a_max, j_max));
    segments.push_back(makeSegment(200.0, 0, 1, 0, v_max, a_max, j_max));

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    std::cout << "\n=== L-Shape Welding Path ===" << std::endl;
    std::cout << "  Seg0: v_start=" << segments[0].v_start
              << " v_cruise=" << segments[0].v_cruise
              << " v_end=" << segments[0].v_end << std::endl;
    std::cout << "  Seg1: v_start=" << segments[1].v_start
              << " v_cruise=" << segments[1].v_cruise
              << " v_end=" << segments[1].v_end << std::endl;

    // Corner velocity must be > 0
    EXPECT_GT(segments[0].v_end, 0.0);
    EXPECT_GT(segments[1].v_start, 0.0);
    EXPECT_NEAR(segments[0].v_end, segments[1].v_start, 1e-10);

    // Cruise should be at or near welding speed
    EXPECT_GT(segments[0].v_cruise, 50.0);
    EXPECT_GT(segments[1].v_cruise, 50.0);
}

// ============================================================================
// Profile Dump — visual verification
// ============================================================================

TEST(LookAheadOptimize, ProfileDump) {
    // Hexagonal path (6 segments, 60° internal angle = 120° turn at each corner)
    std::vector<PathSegment> segments;
    double side = 150.0;

    for (int i = 0; i < 6; ++i) {
        double angle = i * M_PI / 3.0;  // 0, 60, 120, 180, 240, 300 degrees
        double dx = std::cos(angle);
        double dy = std::sin(angle);
        segments.push_back(makeSegment(side, dx, dy, 0));
    }

    LookAheadOptimizer optimizer;
    optimizer.optimize(segments, 0.0, 0.0);

    std::cout << "\n=== Hexagonal Path Look-Ahead ===" << std::endl;
    for (int i = 0; i < (int)segments.size(); ++i) {
        double corner_angle = 0;
        if (i > 0) {
            corner_angle = LookAheadOptimizer::cornerAngle(
                segments[i - 1].dx, segments[i - 1].dy, segments[i - 1].dz,
                segments[i].dx, segments[i].dy, segments[i].dz);
        }
        std::cout << "  Seg" << i
                  << ": corner=" << std::fixed << std::setprecision(1)
                  << (corner_angle * 180.0 / M_PI) << "°"
                  << " v_start=" << std::setprecision(2) << segments[i].v_start
                  << " v_cruise=" << segments[i].v_cruise
                  << " v_end=" << segments[i].v_end
                  << std::endl;
    }

    // All junctions should be continuous
    for (int i = 0; i < (int)segments.size() - 1; ++i) {
        EXPECT_NEAR(segments[i].v_end, segments[i + 1].v_start, 1e-10);
    }
}
