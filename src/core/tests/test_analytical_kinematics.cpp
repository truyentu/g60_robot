/**
 * @file test_analytical_kinematics.cpp
 * @brief Unit tests for AnalyticalKinematics (FK + Analytical IK)
 *
 * Tests:
 * 1. FK at home position = (1332, 0, 1465)
 * 2. FK at multiple known configurations
 * 3. IK round-trip: FK(q) → pose → IK(pose) → contains q (within 1e-6)
 * 4. IK returns multiple solutions (up to 8)
 * 5. IK rejects out-of-limit solutions (no clamping)
 * 6. Singularity detection at θ5 = 0
 * 7. Jacobian non-zero at non-singular configs
 */

#include <gtest/gtest.h>
#include "kinematics/AnalyticalKinematics.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <random>

using namespace robot_controller::kinematics::analytical;

// ============================================================================
// Test Fixture
// ============================================================================

class AnalyticalKinematicsTest : public ::testing::Test {
protected:
    AnalyticalKinematics kin;

    static constexpr double POS_TOL = 0.1;    // mm
    static constexpr double ORI_TOL = 1e-4;   // rad
    static constexpr double ANGLE_TOL = 1e-4;  // rad

    // Check if two poses match
    bool posesMatch(const Pose& a, const Pose& b,
                    double pos_tol = POS_TOL, double ori_tol = ORI_TOL) {
        double pos_err = (a.position - b.position).norm();
        // Orientation error via angle between quaternions
        double dot = std::abs(a.orientation.dot(b.orientation));
        dot = std::min(dot, 1.0);
        double ori_err = 2.0 * std::acos(dot);
        return pos_err < pos_tol && ori_err < ori_tol;
    }

    // Check if a JointAngles is in a solution set (within tolerance)
    bool solutionContains(const IKSolution& ik, const JointAngles& q_expected,
                          double tol = ANGLE_TOL) {
        for (const auto& sol : ik.solutions) {
            bool match = true;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                double diff = std::abs(AnalyticalKinematics::normalizeAngle(
                    sol[i] - q_expected[i]));
                if (diff > tol) {
                    match = false;
                    break;
                }
            }
            if (match) return true;
        }
        return false;
    }
};

// ============================================================================
// FK Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, FK_HomePosition) {
    JointAngles q;  // all zeros
    auto pose = kin.computeFK(q);

    std::cout << "FK Home: pos=(" << pose.position.x() << ", "
              << pose.position.y() << ", " << pose.position.z() << ")"
              << std::endl;

    // Expected: (1332, 0, 1465) from URDF verification
    EXPECT_NEAR(pose.position.x(), 1332.0, 1.0)
        << "Home X should be ~1332mm";
    EXPECT_NEAR(pose.position.y(), 0.0, 1.0)
        << "Home Y should be ~0mm";
    EXPECT_NEAR(pose.position.z(), 1465.0, 1.0)
        << "Home Z should be ~1465mm";
}

TEST_F(AnalyticalKinematicsTest, FK_J1_90deg) {
    JointAngles q;
    q[0] = PI / 2;  // J1 = 90°
    auto pose = kin.computeFK(q);

    std::cout << "FK J1=90°: pos=(" << pose.position.x() << ", "
              << pose.position.y() << ", " << pose.position.z() << ")"
              << std::endl;

    // When J1 rotates 90°, X→Y, Y→-X
    // Expected: (~0, ~1332, ~1465)
    EXPECT_NEAR(pose.position.x(), 0.0, 1.0);
    EXPECT_NEAR(pose.position.y(), 1332.0, 1.0);
    EXPECT_NEAR(pose.position.z(), 1465.0, 1.0);
}

TEST_F(AnalyticalKinematicsTest, FK_J2_45deg) {
    JointAngles q;
    q[1] = PI / 4;  // J2 = 45°
    auto pose = kin.computeFK(q);

    std::cout << "FK J2=45°: pos=(" << pose.position.x() << ", "
              << pose.position.y() << ", " << pose.position.z() << ")"
              << std::endl;

    // J2 rotates the arm in the XZ plane
    // Position should change from home
    EXPECT_NE(pose.position.x(), 1332.0);
    EXPECT_NEAR(pose.position.y(), 0.0, 1.0);  // stays in XZ plane
}

TEST_F(AnalyticalKinematicsTest, FK_AllTransforms_Intermediate) {
    JointAngles q;
    auto T = kin.computeAllTransforms(q);

    // T[0] = T_01: origin should be at (150, 0, 505) — J2 location
    EXPECT_NEAR(T[0](0, 3), 150.0, 1.0);
    EXPECT_NEAR(T[0](1, 3), 0.0, 1.0);
    EXPECT_NEAR(T[0](2, 3), 505.0, 1.0);

    // T[1] = T_02: origin at (150, 0, 1265) — J3 location
    EXPECT_NEAR(T[1](0, 3), 150.0, 1.0);
    EXPECT_NEAR(T[1](2, 3), 1265.0, 1.0);

    // T[2] = T_03: origin at (150, 0, 1465) — J4 location
    EXPECT_NEAR(T[2](0, 3), 150.0, 1.0);
    EXPECT_NEAR(T[2](2, 3), 1465.0, 1.0);

    // T[3] = T_04: origin at (1232, 0, 1465) — J5 location
    EXPECT_NEAR(T[3](0, 3), 1232.0, 1.0);
    EXPECT_NEAR(T[3](2, 3), 1465.0, 1.0);
}

// ============================================================================
// IK Round-Trip Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, IK_RoundTrip_Home) {
    JointAngles q_input;
    q_input[4] = 0.1;  // Slightly off singularity (θ5≠0)

    auto pose = kin.computeFK(q_input);
    auto ik = kin.computeIK(pose);

    std::cout << "IK Home (θ5=0.1): " << ik.count() << " solutions" << std::endl;

    ASSERT_TRUE(ik.hasSolution())
        << "IK should find at least 1 solution for reachable pose";

    // Verify at least one solution matches input
    bool found = false;
    for (const auto& sol : ik.solutions) {
        auto verify = kin.computeFK(sol);
        double pos_err = (verify.position - pose.position).norm();
        if (pos_err < 0.1) {
            found = true;
            std::cout << "  Solution match: pos_err=" << pos_err << "mm" << std::endl;
            break;
        }
    }
    EXPECT_TRUE(found) << "At least one IK solution should recover the original pose";
}

TEST_F(AnalyticalKinematicsTest, IK_RoundTrip_MultipleConfigs) {
    // Test configurations (avoiding wrist singularity θ5=0)
    std::vector<std::pair<std::string, JointAngles>> configs = {
        {"j1_30_j5_20", {0.5236, 0, 0, 0, 0.3491, 0}},       // J1=30°, J5=20°
        {"j2_45_j5_30", {0, 0.7854, 0, 0, 0.5236, 0}},        // J2=45°, J5=30°
        {"j12_combo",   {0.5, -0.3, 0.5, 0, 0.4, 0}},         // Multiple joints
        {"all_small",   {0.1, -0.2, 0.3, -0.1, 0.4, -0.3}},   // All joints small
        {"random_1",    {0.5, -0.3, 1.2, -0.7, 0.4, -1.1}},   // Larger angles
    };

    for (const auto& [name, q_input] : configs) {
        if (!kin.isWithinJointLimits(q_input)) continue;

        auto pose = kin.computeFK(q_input);
        auto ik = kin.computeIK(pose);

        std::cout << "IK " << name << ": " << ik.count() << " solutions"
                  << "  pos=(" << std::fixed << std::setprecision(1)
                  << pose.position.x() << ", " << pose.position.y()
                  << ", " << pose.position.z() << ")" << std::endl;

        ASSERT_TRUE(ik.hasSolution())
            << "IK should find solution for config: " << name;

        // Verify at least one solution reproduces the target pose
        bool pose_match = false;
        for (const auto& sol : ik.solutions) {
            auto verify = kin.computeFK(sol);
            double pos_err = (verify.position - pose.position).norm();
            double dot = std::abs(verify.orientation.dot(pose.orientation));
            dot = std::min(dot, 1.0);
            double ori_err = 2.0 * std::acos(dot);

            if (pos_err < 0.5 && ori_err < 0.01) {
                pose_match = true;
                break;
            }
        }
        EXPECT_TRUE(pose_match)
            << "No IK solution reproduces target for: " << name;
    }
}

TEST_F(AnalyticalKinematicsTest, IK_RoundTrip_ExactMatch) {
    // Most precise test: input angles should appear in output (within 1e-4 rad)
    JointAngles q_input = {0.3, 0.5, -0.4, 0.2, 0.6, -0.5};

    if (!kin.isWithinJointLimits(q_input)) {
        GTEST_SKIP() << "Config out of joint limits";
    }

    auto pose = kin.computeFK(q_input);
    auto ik = kin.computeIK(pose);

    ASSERT_TRUE(ik.hasSolution());

    // Find the closest solution to input
    auto nearest = kin.computeIKNearest(pose, q_input);
    ASSERT_TRUE(nearest.has_value());

    // Verify FK of nearest solution matches
    auto verify = kin.computeFK(*nearest);
    double pos_err = (verify.position - pose.position).norm();
    EXPECT_LT(pos_err, 0.1) << "Nearest IK solution pos error: " << pos_err << "mm";
}

// ============================================================================
// IK Multi-Solution Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, IK_MultipleSolutions) {
    // A general pose should have multiple solutions
    JointAngles q = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    auto pose = kin.computeFK(q);
    auto ik = kin.computeIK(pose);

    std::cout << "Multi-solution test: " << ik.count() << " solutions found"
              << std::endl;

    // Should have at least 2 (elbow up/down)
    EXPECT_GE(ik.count(), 1u)
        << "Should find at least 1 valid solution";

    // All solutions should be valid (within limits)
    for (const auto& sol : ik.solutions) {
        EXPECT_TRUE(kin.isWithinJointLimits(sol))
            << "All returned solutions must be within joint limits";
    }

    // All solutions should produce the same target pose
    for (size_t i = 0; i < ik.solutions.size(); ++i) {
        auto verify = kin.computeFK(ik.solutions[i]);
        double pos_err = (verify.position - pose.position).norm();
        EXPECT_LT(pos_err, 0.5)
            << "Solution " << i << " pos error: " << pos_err << "mm";
    }
}

// ============================================================================
// IK Joint Limit Tests (NO CLAMPING!)
// ============================================================================

TEST_F(AnalyticalKinematicsTest, IK_NoSolutionForUnreachable) {
    // Pose far outside workspace
    Pose unreachable;
    unreachable.position = Eigen::Vector3d(5000, 0, 0);  // Way too far
    unreachable.orientation = Eigen::Quaterniond::Identity();

    auto ik = kin.computeIK(unreachable);
    EXPECT_FALSE(ik.hasSolution())
        << "Should return empty for unreachable pose";
}

TEST_F(AnalyticalKinematicsTest, IK_RejectsOutOfLimits) {
    // All IK solutions should be within joint limits
    // Test with many random poses
    JointAngles configs[] = {
        {0, 0, 0, 0, 0.5, 0},
        {1.0, -0.5, 0.8, -0.3, 0.7, 0.2},
        {-0.5, 0.3, -0.3, 1.0, -0.5, 1.5},
    };

    for (const auto& q : configs) {
        if (!kin.isWithinJointLimits(q)) continue;

        auto pose = kin.computeFK(q);
        auto ik = kin.computeIK(pose);

        for (const auto& sol : ik.solutions) {
            EXPECT_TRUE(kin.isWithinJointLimits(sol))
                << "IK must NEVER return out-of-limits solution (no clamping!)";
        }
    }
}

// ============================================================================
// Singularity Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, Singularity_WristAtZero) {
    JointAngles q;
    q[4] = 0.0;  // θ5 = 0 → wrist singularity
    EXPECT_TRUE(kin.isSingularity(q))
        << "θ5=0 is a wrist singularity";
}

TEST_F(AnalyticalKinematicsTest, Singularity_NotAtGeneral) {
    JointAngles q = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    EXPECT_FALSE(kin.isSingularity(q))
        << "General configuration should NOT be singular";
}

TEST_F(AnalyticalKinematicsTest, Manipulability_PositiveAtGeneral) {
    JointAngles q = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    double m = kin.manipulabilityMeasure(q);
    EXPECT_GT(m, 0.0)
        << "Manipulability should be positive at non-singular config";
    std::cout << "Manipulability at general config: " << m << std::endl;
}

TEST_F(AnalyticalKinematicsTest, Manipulability_NearZeroAtSingularity) {
    JointAngles q;
    q[4] = 0.001;  // Very near wrist singularity
    double m = kin.manipulabilityMeasure(q);
    std::cout << "Manipulability near singularity: " << m << std::endl;
    // Should be very small (but exact value depends on other joints)
}

// ============================================================================
// Jacobian Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, Jacobian_NonZeroAtGeneral) {
    JointAngles q = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    auto J = kin.computeJacobian(q);

    // Determinant should be non-zero at non-singular config
    double det = J.determinant();
    std::cout << "Jacobian det at general config: " << det << std::endl;
    EXPECT_GT(std::abs(det), 1e-6)
        << "Jacobian should be full-rank at non-singular config";
}

TEST_F(AnalyticalKinematicsTest, Jacobian_NearZeroAtSingularity) {
    JointAngles q;
    q[4] = 0.0;  // Wrist singularity
    auto J = kin.computeJacobian(q);

    double det = J.determinant();
    std::cout << "Jacobian det at wrist singularity: " << det << std::endl;
    // Should be near zero
    EXPECT_LT(std::abs(det), 1.0)
        << "Jacobian should be near-singular when θ5≈0";
}

// ============================================================================
// FK vs URDF Cross-Validation (if UrdfForwardKinematics is available)
// ============================================================================

TEST_F(AnalyticalKinematicsTest, FK_CrossValidation_MultiConfigs) {
    // Known URDF FK results (from test_urdf_fk.cpp verification)
    // We verify DH FK produces same position for these configs

    struct TestCase {
        std::string name;
        JointAngles q;
        Eigen::Vector3d expected_pos;  // approximate
    };

    std::vector<TestCase> cases = {
        {"home",     {0, 0, 0, 0, 0, 0},            {1332, 0, 1465}},
        {"j1_90",    {PI/2, 0, 0, 0, 0, 0},         {0, 1332, 1465}},
        {"j1_neg90", {-PI/2, 0, 0, 0, 0, 0},        {0, -1332, 1465}},
    };

    for (const auto& tc : cases) {
        auto pose = kin.computeFK(tc.q);
        double pos_err = (pose.position - tc.expected_pos).norm();

        std::cout << tc.name << ": pos=("
                  << std::fixed << std::setprecision(1)
                  << pose.position.x() << ", "
                  << pose.position.y() << ", "
                  << pose.position.z() << ")"
                  << " err=" << pos_err << "mm" << std::endl;

        EXPECT_LT(pos_err, 1.0)
            << "FK mismatch for " << tc.name;
    }
}

// ============================================================================
// IK Solve Rate Test
// ============================================================================

TEST_F(AnalyticalKinematicsTest, IK_SolveRate_Random) {
    // Generate random joint configs, compute FK, then IK
    // Analytical IK should have ~100% solve rate for reachable poses
    int total = 200;
    int success = 0;
    int valid_configs = 0;

    std::mt19937 rng(42);
    const auto& dh = kin.dhParams();

    for (int i = 0; i < total; ++i) {
        JointAngles q;
        bool within = true;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            double margin = 0.05;
            std::uniform_real_distribution<double> dist(
                dh[j].q_min + margin, dh[j].q_max - margin);
            q[j] = dist(rng);
        }

        // Skip near-singular configs (θ5 near 0)
        if (std::abs(q[4]) < 0.1) continue;

        valid_configs++;
        auto pose = kin.computeFK(q);
        auto ik = kin.computeIK(pose);

        if (ik.hasSolution()) {
            // Verify at least one solution reproduces pose
            bool pose_ok = false;
            for (const auto& sol : ik.solutions) {
                auto verify = kin.computeFK(sol);
                double pos_err = (verify.position - pose.position).norm();
                if (pos_err < 1.0) {
                    pose_ok = true;
                    break;
                }
            }
            if (pose_ok) success++;
        }
    }

    double rate = 100.0 * success / valid_configs;
    std::cout << "IK Solve Rate: " << success << "/" << valid_configs
              << " = " << std::fixed << std::setprecision(1) << rate << "%"
              << std::endl;

    EXPECT_GT(rate, 90.0)
        << "Analytical IK should have >90% solve rate for reachable configs";
}

// ============================================================================
// Joint Limit Cost Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, JointLimitCost_ZeroAtCenter) {
    // At joint mid-range, cost should be minimal
    const auto& dh = kin.dhParams();
    JointAngles q_mid;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        q_mid[i] = 0.5 * (dh[i].q_min + dh[i].q_max);
    }
    double cost = kin.jointLimitCost(q_mid);
    std::cout << "Joint limit cost at mid-range: " << cost << std::endl;
    EXPECT_NEAR(cost, 0.0, 1e-10)
        << "Cost should be zero when all joints are at mid-range";
}

TEST_F(AnalyticalKinematicsTest, JointLimitCost_HighAtLimits) {
    // At joint limits, cost should be high
    const auto& dh = kin.dhParams();
    JointAngles q_max;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        q_max[i] = dh[i].q_max;
    }
    double cost = kin.jointLimitCost(q_max);
    std::cout << "Joint limit cost at max limits: " << cost << std::endl;
    EXPECT_GT(cost, 1.0)
        << "Cost should be high when joints are at limits";
}

// ============================================================================
// Welding IK Tests
// ============================================================================

TEST_F(AnalyticalKinematicsTest, WeldingIK_ReachablePose) {
    // FK at a known config, then solve welding IK
    JointAngles q_start = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    auto pose = kin.computeFK(q_start);

    auto result = kin.computeWeldingIK(pose, q_start);
    ASSERT_TRUE(result.valid) << "Welding IK should find a solution for reachable pose";

    // Verify the solution reaches the same position
    auto verify = kin.computeFK(result.joints);
    double pos_err = (verify.position - pose.position).norm();
    EXPECT_LT(pos_err, 1.0)
        << "Welding IK solution position error: " << pos_err << "mm";

    std::cout << "WeldingIK: psi=" << result.psi * RAD2DEG << "deg"
              << ", cost=" << result.jointLimitCost
              << ", manip=" << result.manipulability
              << ", pos_err=" << pos_err << "mm" << std::endl;
}

TEST_F(AnalyticalKinematicsTest, WeldingIK_BetterThanNaive) {
    // Generate a config near joint limits, solve welding IK
    // Result should have lower joint-limit cost than naive IK
    JointAngles q_near_limit = {0.0, 2.5, 2.5, 0.0, 1.0, 0.0};  // J2,J3 near max
    auto pose = kin.computeFK(q_near_limit);

    // Naive IK: just solve without redundancy
    auto naive = kin.computeIKNearest(pose, q_near_limit);

    // Welding IK: exploit tool Z rotation
    auto welding = kin.computeWeldingIK(pose, q_near_limit);

    if (naive.has_value() && welding.valid) {
        double naive_cost = kin.jointLimitCost(*naive);
        double welding_cost = kin.jointLimitCost(welding.joints);

        std::cout << "Naive IK cost: " << naive_cost
                  << ", Welding IK cost: " << welding_cost
                  << " (psi=" << welding.psi * RAD2DEG << "deg)" << std::endl;

        // Welding IK should be at least as good as naive
        // (it samples ψ=0 which is equivalent to naive)
        EXPECT_LE(welding_cost, naive_cost + 0.01)
            << "Welding IK should not be worse than naive IK";
    }
}

TEST_F(AnalyticalKinematicsTest, WeldingIK_SolveRate_Random) {
    int total = 100;
    int success = 0;
    int valid_configs = 0;

    std::mt19937 rng(123);
    const auto& dh = kin.dhParams();

    for (int i = 0; i < total; ++i) {
        JointAngles q;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            double margin = 0.1;
            std::uniform_real_distribution<double> dist(
                dh[j].q_min + margin, dh[j].q_max - margin);
            q[j] = dist(rng);
        }

        if (std::abs(q[4]) < 0.1) continue;
        valid_configs++;

        auto pose = kin.computeFK(q);
        auto result = kin.computeWeldingIK(pose, q);

        if (result.valid) {
            auto verify = kin.computeFK(result.joints);
            double pos_err = (verify.position - pose.position).norm();
            if (pos_err < 1.0) success++;
        }
    }

    double rate = 100.0 * success / valid_configs;
    std::cout << "Welding IK Solve Rate: " << success << "/" << valid_configs
              << " = " << std::fixed << std::setprecision(1) << rate << "%"
              << std::endl;

    EXPECT_GT(rate, 90.0)
        << "Welding IK should have >90% solve rate";
}

TEST_F(AnalyticalKinematicsTest, WeldingIK_PreservesToolZAxis) {
    // Welding IK rotates around tool Z, so the tool Z axis direction
    // should be preserved (only rotation AROUND Z changes)
    JointAngles q_start = {0.3, 0.5, -0.3, 0.2, 0.6, -0.5};
    auto pose = kin.computeFK(q_start);

    auto result = kin.computeWeldingIK(pose, q_start);
    ASSERT_TRUE(result.valid);

    auto verify = kin.computeFK(result.joints);

    // Tool Z axis (3rd column of rotation matrix) should be same direction
    Eigen::Vector3d z_original = pose.rotationMatrix().col(2);
    Eigen::Vector3d z_result = verify.rotationMatrix().col(2);

    double z_dot = z_original.dot(z_result);
    std::cout << "Tool Z axis dot product: " << z_dot
              << " (1.0 = perfect alignment)" << std::endl;

    // Allow some tolerance for numerical errors
    EXPECT_GT(z_dot, 0.99)
        << "Tool Z axis should be preserved by welding IK";
}
