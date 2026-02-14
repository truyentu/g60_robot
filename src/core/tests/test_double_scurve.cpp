/**
 * @file test_double_scurve.cpp
 * @brief Unit tests for DoubleSCurveProfile (7-segment S-curve)
 *
 * Tests:
 * 1. Full profile: 0 → v_max → 0 with sufficient distance
 * 2. Short distance: v_max not reached (T4=0, reduced V_lim)
 * 3. Very short distance: a_max not reached (T2=T4=T6=0)
 * 4. Boundary conditions: position, velocity, acceleration continuity
 * 5. Zero distance: trivial case
 * 6. Non-zero start/end velocity
 * 7. Negative direction
 * 8. Exact displacement match
 * 9. Jerk constraint verification
 */

#include <gtest/gtest.h>
#include "trajectory/DoubleSCurveProfile.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace robot_controller::trajectory;

// ============================================================================
// Test Fixture
// ============================================================================

class DoubleSCurveTest : public ::testing::Test {
protected:
    DoubleSCurveProfile profile;

    // Typical MA2010 motion limits
    static constexpr double V_MAX = 500.0;    // mm/s
    static constexpr double A_MAX = 2000.0;   // mm/s²
    static constexpr double J_MAX = 10000.0;  // mm/s³

    // Tolerances
    static constexpr double POS_TOL = 0.01;   // mm
    static constexpr double VEL_TOL = 0.1;    // mm/s
    static constexpr double ACC_TOL = 1.0;    // mm/s²
};

// ============================================================================
// Basic Full Profile Tests
// ============================================================================

TEST_F(DoubleSCurveTest, FullProfile_0_to_vmax_to_0) {
    // Long distance: full 7-segment profile expected
    double q0 = 0.0, q1 = 200.0;  // 200mm travel

    ASSERT_TRUE(profile.plan(q0, q1, 0, 0, V_MAX, A_MAX, J_MAX));
    EXPECT_TRUE(profile.isValid());
    EXPECT_GT(profile.getDuration(), 0);

    double T = profile.getDuration();
    std::cout << "Full profile duration: " << T << "s" << std::endl;
    std::cout << "Peak velocity: " << profile.getPeakVelocity() << " mm/s" << std::endl;

    auto phases = profile.getPhaseDurations();
    std::cout << "Phases: ";
    for (int i = 0; i < 7; ++i) {
        std::cout << "T" << (i+1) << "=" << std::fixed << std::setprecision(4) << phases[i] << " ";
    }
    std::cout << std::endl;

    // Start conditions: pos=0, vel=0, acc=0
    auto s0 = profile.evaluate(0);
    EXPECT_NEAR(s0.position, q0, POS_TOL);
    EXPECT_NEAR(s0.velocity, 0, VEL_TOL);

    // End conditions: pos=q1, vel=0, acc=0
    auto sEnd = profile.evaluate(T);
    EXPECT_NEAR(sEnd.position, q1, POS_TOL)
        << "End position mismatch: expected " << q1 << ", got " << sEnd.position;
    EXPECT_NEAR(sEnd.velocity, 0, VEL_TOL);
    EXPECT_NEAR(sEnd.acceleration, 0, ACC_TOL);
}

TEST_F(DoubleSCurveTest, FullProfile_LongDistance) {
    // Very long distance: cruise phase should be dominant
    double q0 = 0, q1 = 1000;

    ASSERT_TRUE(profile.plan(q0, q1, 0, 0, V_MAX, A_MAX, J_MAX));

    auto phases = profile.getPhaseDurations();
    double T4 = phases[3];  // cruise phase

    std::cout << "Long distance cruise T4: " << T4 << "s" << std::endl;
    EXPECT_GT(T4, 0) << "Cruise phase should exist for long distance";
    EXPECT_NEAR(profile.getPeakVelocity(), V_MAX, VEL_TOL)
        << "Should reach V_MAX for long distance";

    // Verify position at end
    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
}

// ============================================================================
// Degenerate Case Tests
// ============================================================================

TEST_F(DoubleSCurveTest, ShortDistance_NoMaxVelocity) {
    // Short distance: V_max not reached, T4=0
    double q0 = 0, q1 = 5.0;  // very short

    ASSERT_TRUE(profile.plan(q0, q1, 0, 0, V_MAX, A_MAX, J_MAX));

    auto phases = profile.getPhaseDurations();
    double T4 = phases[3];

    std::cout << "Short distance: T4=" << T4
              << ", V_lim=" << profile.getPeakVelocity() << " mm/s" << std::endl;

    EXPECT_NEAR(T4, 0, 1e-6) << "No cruise phase for short distance";
    EXPECT_LT(profile.getPeakVelocity(), V_MAX)
        << "Should not reach V_MAX for short distance";

    // Verify exact displacement
    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
}

TEST_F(DoubleSCurveTest, VeryShortDistance_Triangular) {
    // Very short: neither a_max nor v_max reached
    double q0 = 0, q1 = 0.5;

    ASSERT_TRUE(profile.plan(q0, q1, 0, 0, V_MAX, A_MAX, J_MAX));

    auto phases = profile.getPhaseDurations();
    std::cout << "Very short: V_lim=" << profile.getPeakVelocity()
              << ", T=" << profile.getDuration() << "s" << std::endl;

    // Verify end position
    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
}

TEST_F(DoubleSCurveTest, ZeroDistance) {
    ASSERT_TRUE(profile.plan(0, 0, 0, 0, V_MAX, A_MAX, J_MAX));
    EXPECT_NEAR(profile.getDuration(), 0, 1e-10);
}

// ============================================================================
// Direction Tests
// ============================================================================

TEST_F(DoubleSCurveTest, NegativeDirection) {
    double q0 = 100, q1 = 0;  // backward motion

    ASSERT_TRUE(profile.plan(q0, q1, 0, 0, V_MAX, A_MAX, J_MAX));
    EXPECT_GT(profile.getDuration(), 0);

    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL)
        << "Should reach target in negative direction";

    // Velocity should be negative during motion
    auto sMid = profile.evaluate(profile.getDuration() / 2.0);
    EXPECT_LT(sMid.velocity, 0)
        << "Velocity should be negative for backward motion";
}

// ============================================================================
// Boundary Velocity Tests
// ============================================================================

TEST_F(DoubleSCurveTest, NonZeroStartVelocity) {
    double q0 = 0, q1 = 200;
    double vs = 100;  // start at 100 mm/s

    ASSERT_TRUE(profile.plan(q0, q1, vs, 0, V_MAX, A_MAX, J_MAX));

    auto s0 = profile.evaluate(0);
    EXPECT_NEAR(s0.velocity, vs, VEL_TOL)
        << "Start velocity should be " << vs;

    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
    EXPECT_NEAR(sEnd.velocity, 0, VEL_TOL);
}

TEST_F(DoubleSCurveTest, NonZeroEndVelocity) {
    double q0 = 0, q1 = 200;
    double ve = 50;  // end at 50 mm/s

    ASSERT_TRUE(profile.plan(q0, q1, 0, ve, V_MAX, A_MAX, J_MAX));

    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
    EXPECT_NEAR(sEnd.velocity, ve, VEL_TOL)
        << "End velocity should be " << ve;
}

TEST_F(DoubleSCurveTest, NonZeroBothVelocities) {
    double q0 = 0, q1 = 200;
    double vs = 80, ve = 60;

    ASSERT_TRUE(profile.plan(q0, q1, vs, ve, V_MAX, A_MAX, J_MAX));

    auto s0 = profile.evaluate(0);
    EXPECT_NEAR(s0.velocity, vs, VEL_TOL);

    auto sEnd = profile.evaluate(profile.getDuration());
    EXPECT_NEAR(sEnd.position, q1, POS_TOL);
    EXPECT_NEAR(sEnd.velocity, ve, VEL_TOL);
}

// ============================================================================
// Continuity Tests
// ============================================================================

TEST_F(DoubleSCurveTest, Continuity_Position) {
    // Position must be C0 continuous across all phase boundaries
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 1000;
    double dt = T / N;
    double prevPos = profile.evaluate(0).position;

    for (int i = 1; i <= N; ++i) {
        double t = i * dt;
        double pos = profile.evaluate(t).position;
        double jump = std::abs(pos - prevPos);
        EXPECT_LT(jump, 2.0)  // max step ~ V_MAX * dt
            << "Position jump at t=" << t << ": " << jump;
        prevPos = pos;
    }
}

TEST_F(DoubleSCurveTest, Continuity_Velocity) {
    // Velocity must be C0 continuous (no jumps)
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 1000;
    double dt = T / N;
    double prevVel = profile.evaluate(0).velocity;

    for (int i = 1; i <= N; ++i) {
        double t = i * dt;
        double vel = profile.evaluate(t).velocity;
        double jump = std::abs(vel - prevVel);
        EXPECT_LT(jump, A_MAX * dt * 2.0)  // max change ~ A_MAX * dt
            << "Velocity jump at t=" << t << ": " << jump;
        prevVel = vel;
    }
}

TEST_F(DoubleSCurveTest, Continuity_Acceleration) {
    // Acceleration must be C0 continuous (key property of S-curve!)
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 2000;
    double dt = T / N;
    double prevAcc = profile.evaluate(0).acceleration;

    double maxJump = 0;
    for (int i = 1; i <= N; ++i) {
        double t = i * dt;
        double acc = profile.evaluate(t).acceleration;
        double jump = std::abs(acc - prevAcc);
        maxJump = std::max(maxJump, jump);
        prevAcc = acc;
    }

    std::cout << "Max acceleration jump: " << maxJump << " mm/s^2" << std::endl;
    EXPECT_LT(maxJump, J_MAX * dt * 2.0)
        << "Acceleration must be continuous (S-curve property)";
}

// ============================================================================
// Constraint Verification Tests
// ============================================================================

TEST_F(DoubleSCurveTest, VelocityLimit) {
    ASSERT_TRUE(profile.plan(0, 1000, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 500;
    double dt = T / N;
    double maxVel = 0;

    for (int i = 0; i <= N; ++i) {
        double t = i * dt;
        double vel = std::abs(profile.evaluate(t).velocity);
        maxVel = std::max(maxVel, vel);
    }

    std::cout << "Max velocity: " << maxVel << " mm/s (limit: " << V_MAX << ")" << std::endl;
    EXPECT_LE(maxVel, V_MAX + VEL_TOL)
        << "Velocity must not exceed V_MAX";
}

TEST_F(DoubleSCurveTest, AccelerationLimit) {
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 500;
    double dt = T / N;
    double maxAcc = 0;

    for (int i = 0; i <= N; ++i) {
        double t = i * dt;
        double acc = std::abs(profile.evaluate(t).acceleration);
        maxAcc = std::max(maxAcc, acc);
    }

    std::cout << "Max acceleration: " << maxAcc << " mm/s^2 (limit: " << A_MAX << ")" << std::endl;
    EXPECT_LE(maxAcc, A_MAX + ACC_TOL)
        << "Acceleration must not exceed A_MAX";
}

TEST_F(DoubleSCurveTest, JerkLimit) {
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 500;
    double dt = T / N;
    double maxJerk = 0;

    for (int i = 0; i <= N; ++i) {
        double t = i * dt;
        double jerk = std::abs(profile.evaluate(t).jerk);
        maxJerk = std::max(maxJerk, jerk);
    }

    std::cout << "Max jerk: " << maxJerk << " mm/s^3 (limit: " << J_MAX << ")" << std::endl;
    EXPECT_LE(maxJerk, J_MAX + 1.0)
        << "Jerk must not exceed J_MAX";
}

// ============================================================================
// Monotonicity Test
// ============================================================================

TEST_F(DoubleSCurveTest, PositionMonotonicallyIncreasing) {
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 500;
    double dt = T / N;
    double prevPos = profile.evaluate(0).position;

    for (int i = 1; i <= N; ++i) {
        double t = i * dt;
        double pos = profile.evaluate(t).position;
        EXPECT_GE(pos, prevPos - POS_TOL)
            << "Position must be monotonically increasing at t=" << t;
        prevPos = pos;
    }
}

TEST_F(DoubleSCurveTest, VelocityNonNegative) {
    // For forward motion, velocity should stay >= 0
    ASSERT_TRUE(profile.plan(0, 200, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    int N = 500;
    double dt = T / N;

    for (int i = 0; i <= N; ++i) {
        double t = i * dt;
        double vel = profile.evaluate(t).velocity;
        EXPECT_GE(vel, -VEL_TOL)
            << "Velocity must be non-negative for forward motion at t=" << t;
    }
}

// ============================================================================
// Profile Dump (Visual Inspection)
// ============================================================================

TEST_F(DoubleSCurveTest, ProfileDump_Visual) {
    ASSERT_TRUE(profile.plan(0, 100, 0, 0, V_MAX, A_MAX, J_MAX));
    double T = profile.getDuration();

    std::cout << "\n=== S-Curve Profile Dump ===" << std::endl;
    std::cout << "Duration: " << T << "s" << std::endl;
    std::cout << "V_lim: " << profile.getPeakVelocity() << " mm/s" << std::endl;

    auto phases = profile.getPhaseDurations();
    std::cout << "Phases: ";
    for (int i = 0; i < 7; ++i) {
        std::cout << "T" << (i+1) << "=" << std::fixed << std::setprecision(5) << phases[i] << " ";
    }
    std::cout << std::endl;

    std::cout << std::setw(8) << "Time"
              << std::setw(12) << "Pos"
              << std::setw(12) << "Vel"
              << std::setw(12) << "Acc"
              << std::setw(12) << "Jerk" << std::endl;

    int N = 20;
    for (int i = 0; i <= N; ++i) {
        double t = T * i / N;
        auto s = profile.evaluate(t);
        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(8) << t
                  << std::setw(12) << s.position
                  << std::setw(12) << s.velocity
                  << std::setw(12) << s.acceleration
                  << std::setw(12) << s.jerk << std::endl;
    }
}

// ============================================================================
// Edge Case: Input Validation
// ============================================================================

TEST_F(DoubleSCurveTest, InvalidInputs) {
    // Zero or negative limits
    EXPECT_FALSE(profile.plan(0, 100, 0, 0, 0, A_MAX, J_MAX));
    EXPECT_FALSE(profile.plan(0, 100, 0, 0, V_MAX, 0, J_MAX));
    EXPECT_FALSE(profile.plan(0, 100, 0, 0, V_MAX, A_MAX, 0));

    // Negative velocities
    EXPECT_FALSE(profile.plan(0, 100, -1, 0, V_MAX, A_MAX, J_MAX));
    EXPECT_FALSE(profile.plan(0, 100, 0, -1, V_MAX, A_MAX, J_MAX));

    // Start velocity > v_max
    EXPECT_FALSE(profile.plan(0, 100, V_MAX + 1, 0, V_MAX, A_MAX, J_MAX));
}

// ============================================================================
// Stress Test: Multiple Distances
// ============================================================================

TEST_F(DoubleSCurveTest, MultipleDistances_AllReachTarget) {
    double distances[] = {0.1, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};

    int pass = 0, total = 0;
    for (double d : distances) {
        total++;
        if (!profile.plan(0, d, 0, 0, V_MAX, A_MAX, J_MAX)) {
            std::cout << "FAIL plan for d=" << d << std::endl;
            continue;
        }

        auto sEnd = profile.evaluate(profile.getDuration());
        double err = std::abs(sEnd.position - d);

        if (err < 0.1) {
            pass++;
        } else {
            std::cout << "FAIL d=" << d << " pos_err=" << err << std::endl;
        }
    }

    std::cout << "Distance test: " << pass << "/" << total << " passed" << std::endl;
    EXPECT_EQ(pass, total);
}
