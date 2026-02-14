/**
 * @file test_ma2010_critical_geometry.cpp
 * @brief Critical verification of h3/h4 sign (Section 5.4) and theta5 negation (Section 6)
 *
 * Check 1: "Elbow L-Shape" — q3 = -85° → IK must recover q3 = -85° (not +85°)
 *           (q3_min = -86°, so -90° is out of limits; -85° is the max valid test)
 * Check 2: "Wrist Pitch"  — q5 = +45° → IK must recover q5 = +45° (not -45°)
 */

#include <gtest/gtest.h>
#include "kinematics/AnalyticalKinematics.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace robot_controller::kinematics::analytical;

class CriticalGeometryTest : public ::testing::Test {
protected:
    AnalyticalKinematics kin;
};

// ============================================================================
// Check 1: Elbow L-Shape (Verify Section 5.4 — h3/h4 sign for alpha1=-90°)
// ============================================================================

TEST_F(CriticalGeometryTest, ElbowLShape_Q3_Neg85) {
    // q_urdf = [0, 0, -85°, 0, 0.1, 0]
    // Note: q3_min = -86° (= -1.5009 rad), so -90° is OUT of limits.
    //       We use -85° as the max-deflection test within limits.
    // Note: q5=0.1 to avoid wrist singularity (theta5=0 is degenerate)
    //
    // PURPOSE: If h3/h4 signs are wrong, IK will return +85° instead of -85°.
    constexpr double Q3_TEST_DEG = -85.0;
    JointAngles q_input;
    q_input[0] = 0.0;
    q_input[1] = 0.0;
    q_input[2] = Q3_TEST_DEG * DEG2RAD;
    q_input[3] = 0.0;
    q_input[4] = 0.1;              // slight offset from singularity
    q_input[5] = 0.0;

    ASSERT_TRUE(kin.isWithinJointLimits(q_input))
        << "Input config must be within joint limits";

    // Step 1: FK
    auto pose = kin.computeFK(q_input);
    std::cout << "=== Check 1: Elbow L-Shape (q3 = " << Q3_TEST_DEG << "°) ===" << std::endl;
    std::cout << "Input q3 = " << q_input[2] * RAD2DEG << " deg" << std::endl;
    std::cout << "FK position: (" << std::fixed << std::setprecision(2)
              << pose.position.x() << ", "
              << pose.position.y() << ", "
              << pose.position.z() << ") mm" << std::endl;

    // Step 2: IK
    auto ik = kin.computeIK(pose);
    ASSERT_TRUE(ik.hasSolution())
        << "IK must find at least 1 solution for reachable pose";

    std::cout << "IK found " << ik.count() << " solutions" << std::endl;

    // Step 3: Find a solution that matches q_input
    bool found_correct_q3 = false;
    bool found_wrong_q3 = false;

    for (size_t i = 0; i < ik.solutions.size(); ++i) {
        const auto& sol = ik.solutions[i];
        double q3_deg = sol[2] * RAD2DEG;

        // Verify this solution reproduces the pose
        auto verify = kin.computeFK(sol);
        double pos_err = (verify.position - pose.position).norm();

        std::cout << "  Solution " << i << ": q3 = " << std::setprecision(2)
                  << q3_deg << " deg"
                  << ", pos_err = " << std::setprecision(4) << pos_err << " mm"
                  << ", q = [";
        for (int j = 0; j < NUM_JOINTS; ++j) {
            std::cout << std::setprecision(2) << sol[j] * RAD2DEG;
            if (j < 5) std::cout << ", ";
        }
        std::cout << "] deg" << std::endl;

        if (pos_err < 0.5) {
            double q3_diff = std::abs(AnalyticalKinematics::normalizeAngle(
                sol[2] - q_input[2]));

            if (q3_diff < 0.01) {  // within ~0.6°
                found_correct_q3 = true;
            }

            // Check if wrong sign (+90° instead of -90°)
            double wrong_diff = std::abs(AnalyticalKinematics::normalizeAngle(
                sol[2] - (-q_input[2])));
            if (wrong_diff < 0.01) {
                found_wrong_q3 = true;
            }
        }
    }

    EXPECT_TRUE(found_correct_q3)
        << "CRITICAL: IK must recover q3 = " << Q3_TEST_DEG << " deg. "
        << "If it returns +" << -Q3_TEST_DEG << " deg instead, h3/h4 signs are WRONG.";

    EXPECT_FALSE(found_wrong_q3)
        << "WARNING: IK also returned q3 = +" << -Q3_TEST_DEG << " deg — suspicious sign error";

    // Step 4: Use computeIKNearest to verify it picks the right one
    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    double q3_nearest_deg = (*nearest)[2] * RAD2DEG;
    std::cout << "IKNearest q3 = " << q3_nearest_deg << " deg (expected " << Q3_TEST_DEG << ")" << std::endl;

    EXPECT_NEAR((*nearest)[2], Q3_TEST_DEG * DEG2RAD, 0.01)
        << "IKNearest must return q3 ≈ " << Q3_TEST_DEG << "° for elbow L-shape config";
}

// Also test with exact q5=0 singularity to see behavior
TEST_F(CriticalGeometryTest, ElbowLShape_Q3_Neg85_ExactSingularity) {
    constexpr double Q3_TEST_DEG = -85.0;
    JointAngles q_input;
    q_input[2] = Q3_TEST_DEG * DEG2RAD;
    // q5 = 0 → wrist singularity, but position sub-problem should still work

    auto pose = kin.computeFK(q_input);
    auto ik = kin.computeIK(pose);

    std::cout << "\n=== Check 1b: Elbow L-Shape + Wrist Singularity ===" << std::endl;
    std::cout << "FK position: (" << std::fixed << std::setprecision(2)
              << pose.position.x() << ", "
              << pose.position.y() << ", "
              << pose.position.z() << ") mm" << std::endl;
    std::cout << "IK found " << ik.count() << " solutions" << std::endl;

    ASSERT_TRUE(ik.hasSolution());

    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    std::cout << "IKNearest q3 = " << (*nearest)[2] * RAD2DEG << " deg" << std::endl;

    // Position sub-problem should still find q3 = -85°
    EXPECT_NEAR((*nearest)[2], Q3_TEST_DEG * DEG2RAD, 0.05)
        << "Position IK must work correctly even at wrist singularity";
}

// ============================================================================
// Check 2: Wrist Pitch (Verify Section 6 — theta5 negation in R_36)
// ============================================================================

TEST_F(CriticalGeometryTest, WristPitch_Q5_Pos45) {
    // q_urdf = [0, 0, 0, 0, +45°, 0]
    // Tool should tilt by 45° around joint 5 axis
    JointAngles q_input;
    q_input[4] = 45.0 * DEG2RAD;  // +π/4

    ASSERT_TRUE(kin.isWithinJointLimits(q_input))
        << "Input config must be within joint limits";

    // Step 1: FK
    auto pose = kin.computeFK(q_input);
    std::cout << "\n=== Check 2: Wrist Pitch (q5 = +45°) ===" << std::endl;
    std::cout << "Input q5 = " << q_input[4] * RAD2DEG << " deg" << std::endl;
    std::cout << "FK position: (" << std::fixed << std::setprecision(2)
              << pose.position.x() << ", "
              << pose.position.y() << ", "
              << pose.position.z() << ") mm" << std::endl;

    auto R = pose.rotationMatrix();
    std::cout << "FK rotation matrix:" << std::endl;
    for (int r = 0; r < 3; ++r) {
        std::cout << "  [";
        for (int c = 0; c < 3; ++c) {
            std::cout << std::setw(10) << std::setprecision(4) << R(r, c);
        }
        std::cout << " ]" << std::endl;
    }

    // Step 2: IK
    auto ik = kin.computeIK(pose);
    ASSERT_TRUE(ik.hasSolution())
        << "IK must find at least 1 solution";

    std::cout << "IK found " << ik.count() << " solutions" << std::endl;

    // Step 3: Check all solutions
    bool found_correct_q5 = false;
    bool found_negated_q5 = false;

    for (size_t i = 0; i < ik.solutions.size(); ++i) {
        const auto& sol = ik.solutions[i];
        double q5_deg = sol[4] * RAD2DEG;

        auto verify = kin.computeFK(sol);
        double pos_err = (verify.position - pose.position).norm();
        double dot = std::abs(verify.orientation.dot(pose.orientation));
        dot = std::min(dot, 1.0);
        double ori_err = 2.0 * std::acos(dot);

        std::cout << "  Solution " << i << ": q5 = " << std::setprecision(2)
                  << q5_deg << " deg"
                  << ", pos_err = " << std::setprecision(4) << pos_err << " mm"
                  << ", ori_err = " << std::setprecision(6) << ori_err << " rad"
                  << ", q = [";
        for (int j = 0; j < NUM_JOINTS; ++j) {
            std::cout << std::setprecision(2) << sol[j] * RAD2DEG;
            if (j < 5) std::cout << ", ";
        }
        std::cout << "] deg" << std::endl;

        if (pos_err < 0.5 && ori_err < 0.01) {
            double q5_diff = std::abs(AnalyticalKinematics::normalizeAngle(
                sol[4] - q_input[4]));
            if (q5_diff < 0.01) {
                found_correct_q5 = true;
            }

            // Check if wrong sign (-45° instead of +45°)
            double wrong_diff = std::abs(AnalyticalKinematics::normalizeAngle(
                sol[4] - (-q_input[4])));
            if (wrong_diff < 0.01) {
                found_negated_q5 = true;
            }
        }
    }

    EXPECT_TRUE(found_correct_q5)
        << "CRITICAL: IK must recover q5 = +45 deg. "
        << "If it returns -45 deg instead, theta5 negation in Section 6 is WRONG.";

    if (found_negated_q5) {
        ADD_FAILURE() << "WARNING: IK returned q5 = -45 deg — theta5 sign is INVERTED!";
    }

    // Step 4: IKNearest verification
    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    double q5_nearest_deg = (*nearest)[4] * RAD2DEG;
    std::cout << "IKNearest q5 = " << q5_nearest_deg << " deg (expected +45)" << std::endl;

    EXPECT_NEAR((*nearest)[4], 45.0 * DEG2RAD, 0.01)
        << "IKNearest must return q5 ≈ +45° for wrist pitch config";
}

// Additional: Test negative q5 to be thorough
TEST_F(CriticalGeometryTest, WristPitch_Q5_Neg60) {
    JointAngles q_input;
    q_input[4] = -60.0 * DEG2RAD;  // -π/3

    ASSERT_TRUE(kin.isWithinJointLimits(q_input));

    auto pose = kin.computeFK(q_input);
    auto ik = kin.computeIK(pose);

    std::cout << "\n=== Check 2b: Wrist Pitch (q5 = -60°) ===" << std::endl;
    std::cout << "IK found " << ik.count() << " solutions" << std::endl;

    ASSERT_TRUE(ik.hasSolution());

    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    double q5_nearest_deg = (*nearest)[4] * RAD2DEG;
    std::cout << "IKNearest q5 = " << q5_nearest_deg << " deg (expected -60)" << std::endl;

    EXPECT_NEAR((*nearest)[4], -60.0 * DEG2RAD, 0.01)
        << "IKNearest must return q5 ≈ -60° for negative wrist pitch";
}

// ============================================================================
// Combined Test: Both q3 and q5 non-zero simultaneously
// ============================================================================

TEST_F(CriticalGeometryTest, Combined_ElbowAndWrist) {
    // q = [30°, 20°, -45°, 10°, 60°, -30°]
    JointAngles q_input;
    q_input[0] = 30.0 * DEG2RAD;
    q_input[1] = 20.0 * DEG2RAD;
    q_input[2] = -45.0 * DEG2RAD;
    q_input[3] = 10.0 * DEG2RAD;
    q_input[4] = 60.0 * DEG2RAD;
    q_input[5] = -30.0 * DEG2RAD;

    ASSERT_TRUE(kin.isWithinJointLimits(q_input));

    auto pose = kin.computeFK(q_input);
    auto ik = kin.computeIK(pose);

    std::cout << "\n=== Check 3: Combined Elbow + Wrist ===" << std::endl;
    std::cout << "Input: q = [30, 20, -45, 10, 60, -30] deg" << std::endl;
    std::cout << "FK position: (" << std::fixed << std::setprecision(2)
              << pose.position.x() << ", "
              << pose.position.y() << ", "
              << pose.position.z() << ") mm" << std::endl;
    std::cout << "IK found " << ik.count() << " solutions" << std::endl;

    ASSERT_TRUE(ik.hasSolution());

    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    std::cout << "IKNearest: q = [";
    for (int j = 0; j < NUM_JOINTS; ++j) {
        std::cout << std::setprecision(2) << (*nearest)[j] * RAD2DEG;
        if (j < 5) std::cout << ", ";
    }
    std::cout << "] deg" << std::endl;

    // All joints should match within tolerance
    for (int j = 0; j < NUM_JOINTS; ++j) {
        double diff = std::abs(AnalyticalKinematics::normalizeAngle(
            (*nearest)[j] - q_input[j]));
        EXPECT_LT(diff, 0.01)
            << "Joint " << (j+1) << " mismatch: got "
            << (*nearest)[j] * RAD2DEG << " deg, expected "
            << q_input[j] * RAD2DEG << " deg";
    }

    // Verify FK of nearest matches target
    auto verify = kin.computeFK(*nearest);
    double pos_err = (verify.position - pose.position).norm();
    EXPECT_LT(pos_err, 0.1) << "FK verification error: " << pos_err << " mm";
}
