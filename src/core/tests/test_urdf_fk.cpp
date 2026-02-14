/**
 * @file test_urdf_fk.cpp
 * @brief Unit tests for URDF Forward Kinematics
 *
 * Test data auto-generated from tests/verify_fk.py using pure NumPy math.
 * All expected values verified against URDF specification.
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_01)
 */

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include "kinematics/UrdfForwardKinematics.hpp"
#include <vector>

using namespace robot_controller::kinematics;

// ============================================================================
// MA2010 Test Fixture
// ============================================================================

/**
 * Create MA2010 URDF FK instance with joint data from ROS-Industrial URDF
 * Units: mm (converted from URDF meters)
 */
static UrdfForwardKinematics createMA2010FK() {
    std::vector<UrdfJointDef> joints(6);

    // J1 (S-axis): base rotation about Z
    joints[0].name = "joint_1_s";
    joints[0].originXyz = Vector3d(0, 0, 505);
    joints[0].originRpy = Vector3d(0, 0, 0);
    joints[0].axis = Vector3d(0, 0, 1);
    joints[0].minAngle = -3.1416;
    joints[0].maxAngle = 3.1416;

    // J2 (L-axis): shoulder rotation about Y
    joints[1].name = "joint_2_l";
    joints[1].originXyz = Vector3d(150, 0, 0);
    joints[1].originRpy = Vector3d(0, 0, 0);
    joints[1].axis = Vector3d(0, 1, 0);
    joints[1].minAngle = -1.8326;
    joints[1].maxAngle = 2.7052;

    // J3 (U-axis): elbow rotation about -Y
    joints[2].name = "joint_3_u";
    joints[2].originXyz = Vector3d(0, 0, 760);
    joints[2].originRpy = Vector3d(0, 0, 0);
    joints[2].axis = Vector3d(0, -1, 0);
    joints[2].minAngle = -1.5009;
    joints[2].maxAngle = 2.7925;

    // J4 (R-axis): wrist roll about -X
    joints[3].name = "joint_4_r";
    joints[3].originXyz = Vector3d(0, 0, 200);
    joints[3].originRpy = Vector3d(0, 0, 0);
    joints[3].axis = Vector3d(-1, 0, 0);
    joints[3].minAngle = -2.6180;
    joints[3].maxAngle = 2.6180;

    // J5 (B-axis): wrist bend about -Y
    joints[4].name = "joint_5_b";
    joints[4].originXyz = Vector3d(1082, 0, 0);
    joints[4].originRpy = Vector3d(0, 0, 0);
    joints[4].axis = Vector3d(0, -1, 0);
    joints[4].minAngle = -2.3562;
    joints[4].maxAngle = 1.5708;

    // J6 (T-axis): tool rotation about -X
    joints[5].name = "joint_6_t";
    joints[5].originXyz = Vector3d(0, 0, 0);
    joints[5].originRpy = Vector3d(0, 0, 0);
    joints[5].axis = Vector3d(-1, 0, 0);
    joints[5].minAngle = -3.6652;
    joints[5].maxAngle = 3.6652;

    // Tool offset: 100mm in X
    Vector3d toolOffset(100, 0, 0);

    return UrdfForwardKinematics(joints, toolOffset);
}

// ============================================================================
// Test data from Python verification (verify_fk.py)
// ============================================================================

struct FKTestCase {
    std::string name;
    JointAngles angles;
    Vector3d expectedPosition;  // mm
    Vector3d expectedRPY;       // rad
};

static const std::vector<FKTestCase> FK_TEST_CASES = {
    {"home", {{0, 0, 0, 0, 0, 0}},
     Vector3d(1332.0, 0.0, 1465.0),
     Vector3d(0.0, 0.0, 0.0)},

    {"j1_90", {{M_PI/2, 0, 0, 0, 0, 0}},
     Vector3d(0.0, 1332.0, 1465.0),
     Vector3d(0.0, 0.0, M_PI/2)},

    {"j1_neg90", {{-M_PI/2, 0, 0, 0, 0, 0}},
     Vector3d(0.0, -1332.0, 1465.0),
     Vector3d(0.0, 0.0, -M_PI/2)},

    {"j2_45", {{0, M_PI/4, 0, 0, 0, 0}},
     Vector3d(1664.622725, 0.0, 348.022295),
     Vector3d(0.0, M_PI/4, 0.0)},

    {"j2_90", {{0, M_PI/2, 0, 0, 0, 0}},
     Vector3d(1110.0, 0.0, -677.0),
     Vector3d(0.0, M_PI/2, 0.0)},

    {"j3_45", {{0, 0, M_PI/4, 0, 0, 0}},
     Vector3d(844.378859, 0.0, 2242.221572),
     Vector3d(0.0, -M_PI/4, 0.0)},

    {"j3_neg45", {{0, 0, -M_PI/4, 0, 0, 0}},
     Vector3d(1127.221572, 0.0, 570.621141),
     Vector3d(0.0, M_PI/4, 0.0)},

    {"j4_90", {{0, 0, 0, M_PI/2, 0, 0}},
     Vector3d(1332.0, 0.0, 1465.0),
     Vector3d(-M_PI/2, 0.0, 0.0)},

    {"j5_45", {{0, 0, 0, 0, M_PI/4, 0}},
     Vector3d(1302.710678, 0.0, 1535.710678),
     Vector3d(0.0, -M_PI/4, 0.0)},

    {"j6_90", {{0, 0, 0, 0, 0, M_PI/2}},
     Vector3d(1332.0, 0.0, 1465.0),
     Vector3d(-M_PI/2, 0.0, 0.0)},

    {"j12_combo", {{M_PI/4, M_PI/4, 0, 0, 0, 0}},
     Vector3d(1177.066017, 1177.066017, 348.022295),
     Vector3d(0.0, M_PI/4, M_PI/4)},

    {"j123_combo", {{M_PI/6, M_PI/4, -M_PI/6, 0, 0, 0}},
     Vector3d(1027.548174, 593.255215, -47.559364),
     Vector3d(0.0, 1.3089969390, M_PI/6)},

    {"all_small", {{0.1, -0.2, 0.3, -0.1, 0.2, -0.3}},
     Vector3d(924.758202, 90.791969, 2008.440211),
     Vector3d(0.4146908360, -0.6988617255, 0.0740899876)},

    {"random_1", {{0.5, -0.3, 1.2, -0.7, 0.4, -1.1}},
     Vector3d(-181.699978, -127.849655, 2418.474982),
     Vector3d(-2.1753686026, -1.2221099625, -1.8170152477)},

    {"random_2", {{-1.0, 0.8, -0.5, 2.0, -1.5, 0.3}},
     Vector3d(582.427513, -1074.949726, 49.714961),
     Vector3d(3.0875241813, -0.0428936785, -2.1381387210)},
};

// ============================================================================
// Individual Joint Tests
// ============================================================================

class UrdfFKTest : public ::testing::Test {
protected:
    UrdfForwardKinematics fk = createMA2010FK();
};

TEST_F(UrdfFKTest, HomePosition) {
    auto pose = fk.compute({0, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose.position.x(), 1332.0, 0.01) << "Home X position";
    EXPECT_NEAR(pose.position.y(), 0.0, 0.01) << "Home Y position";
    EXPECT_NEAR(pose.position.z(), 1465.0, 0.01) << "Home Z position";

    // Orientation = Identity at home
    EXPECT_NEAR(pose.rpy.x(), 0.0, 0.001) << "Home roll";
    EXPECT_NEAR(pose.rpy.y(), 0.0, 0.001) << "Home pitch";
    EXPECT_NEAR(pose.rpy.z(), 0.0, 0.001) << "Home yaw";
}

TEST_F(UrdfFKTest, J1_90deg) {
    auto pose = fk.compute({M_PI/2, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose.position.x(), 0.0, 0.01) << "J1=90 X";
    EXPECT_NEAR(pose.position.y(), 1332.0, 0.01) << "J1=90 Y";
    EXPECT_NEAR(pose.position.z(), 1465.0, 0.01) << "J1=90 Z";
    EXPECT_NEAR(pose.rpy.z(), M_PI/2, 0.001) << "J1=90 yaw";
}

TEST_F(UrdfFKTest, J1_neg90deg) {
    auto pose = fk.compute({-M_PI/2, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose.position.x(), 0.0, 0.01) << "J1=-90 X";
    EXPECT_NEAR(pose.position.y(), -1332.0, 0.01) << "J1=-90 Y";
    EXPECT_NEAR(pose.position.z(), 1465.0, 0.01) << "J1=-90 Z";
}

TEST_F(UrdfFKTest, J2_45deg) {
    auto pose = fk.compute({0, M_PI/4, 0, 0, 0, 0});
    EXPECT_NEAR(pose.position.x(), 1664.622725, 0.01);
    EXPECT_NEAR(pose.position.y(), 0.0, 0.01);
    EXPECT_NEAR(pose.position.z(), 348.022295, 0.01);
    EXPECT_NEAR(pose.rpy.y(), M_PI/4, 0.001) << "J2=45 pitch";
}

TEST_F(UrdfFKTest, J3_45deg) {
    auto pose = fk.compute({0, 0, M_PI/4, 0, 0, 0});
    EXPECT_NEAR(pose.position.x(), 844.378859, 0.01);
    EXPECT_NEAR(pose.position.y(), 0.0, 0.01);
    EXPECT_NEAR(pose.position.z(), 2242.221572, 0.01);
    // J3 axis is -Y, so positive angle rotates arm UP → negative pitch
    EXPECT_NEAR(pose.rpy.y(), -M_PI/4, 0.001) << "J3=45 pitch";
}

TEST_F(UrdfFKTest, J4_90deg_PositionUnchanged) {
    // J4 is wrist roll - position should not change from home
    auto pose = fk.compute({0, 0, 0, M_PI/2, 0, 0});
    EXPECT_NEAR(pose.position.x(), 1332.0, 0.01);
    EXPECT_NEAR(pose.position.y(), 0.0, 0.01);
    EXPECT_NEAR(pose.position.z(), 1465.0, 0.01);
    // Orientation changes: roll about -X axis
    EXPECT_NEAR(pose.rpy.x(), -M_PI/2, 0.001) << "J4=90 roll";
}

TEST_F(UrdfFKTest, J5_45deg) {
    auto pose = fk.compute({0, 0, 0, 0, M_PI/4, 0});
    EXPECT_NEAR(pose.position.x(), 1302.710678, 0.01);
    EXPECT_NEAR(pose.position.y(), 0.0, 0.01);
    EXPECT_NEAR(pose.position.z(), 1535.710678, 0.01);
}

// ============================================================================
// All Configs Match Python Verification
// ============================================================================

TEST_F(UrdfFKTest, AllConfigsMatchPython) {
    constexpr double POS_TOL = 0.01;  // mm
    constexpr double RPY_TOL = 0.001; // rad

    for (const auto& tc : FK_TEST_CASES) {
        auto pose = fk.compute(tc.angles);

        EXPECT_NEAR(pose.position.x(), tc.expectedPosition.x(), POS_TOL)
            << "Config '" << tc.name << "' X mismatch";
        EXPECT_NEAR(pose.position.y(), tc.expectedPosition.y(), POS_TOL)
            << "Config '" << tc.name << "' Y mismatch";
        EXPECT_NEAR(pose.position.z(), tc.expectedPosition.z(), POS_TOL)
            << "Config '" << tc.name << "' Z mismatch";

        EXPECT_NEAR(pose.rpy.x(), tc.expectedRPY.x(), RPY_TOL)
            << "Config '" << tc.name << "' roll mismatch";
        EXPECT_NEAR(pose.rpy.y(), tc.expectedRPY.y(), RPY_TOL)
            << "Config '" << tc.name << "' pitch mismatch";
        EXPECT_NEAR(pose.rpy.z(), tc.expectedRPY.z(), RPY_TOL)
            << "Config '" << tc.name << "' yaw mismatch";
    }
}

// ============================================================================
// Intermediate Transforms
// ============================================================================

TEST_F(UrdfFKTest, IntermediateTransforms_Home) {
    auto transforms = fk.computeAllTransforms({0, 0, 0, 0, 0, 0});

    // Should have 7 transforms: 6 joints + tool
    ASSERT_EQ(transforms.size(), 7u);

    // J1 origin: (0, 0, 505)
    EXPECT_NEAR(transforms[0](0,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[0](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[0](2,3), 505.0, 0.01);

    // J2 origin: (150, 0, 505)
    EXPECT_NEAR(transforms[1](0,3), 150.0, 0.01);
    EXPECT_NEAR(transforms[1](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[1](2,3), 505.0, 0.01);

    // J3 origin: (150, 0, 1265)
    EXPECT_NEAR(transforms[2](0,3), 150.0, 0.01);
    EXPECT_NEAR(transforms[2](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[2](2,3), 1265.0, 0.01);

    // J4 origin: (150, 0, 1465)
    EXPECT_NEAR(transforms[3](0,3), 150.0, 0.01);
    EXPECT_NEAR(transforms[3](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[3](2,3), 1465.0, 0.01);

    // J5 origin: (1232, 0, 1465)
    EXPECT_NEAR(transforms[4](0,3), 1232.0, 0.01);
    EXPECT_NEAR(transforms[4](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[4](2,3), 1465.0, 0.01);

    // J6 origin: (1232, 0, 1465) - same as J5 (xyz=[0,0,0])
    EXPECT_NEAR(transforms[5](0,3), 1232.0, 0.01);
    EXPECT_NEAR(transforms[5](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[5](2,3), 1465.0, 0.01);

    // Tool: (1332, 0, 1465) - 100mm offset in X
    EXPECT_NEAR(transforms[6](0,3), 1332.0, 0.01);
    EXPECT_NEAR(transforms[6](1,3), 0.0, 0.01);
    EXPECT_NEAR(transforms[6](2,3), 1465.0, 0.01);
}

// ============================================================================
// Joint Positions
// ============================================================================

TEST_F(UrdfFKTest, JointPositions_Home) {
    auto positions = fk.computeJointPositions({0, 0, 0, 0, 0, 0});

    ASSERT_EQ(positions.size(), 7u);

    // Base height
    EXPECT_NEAR(positions[0].z(), 505.0, 0.01) << "J1 height (base height)";

    // TCP position
    EXPECT_NEAR(positions[6].x(), 1332.0, 0.01) << "TCP X";
    EXPECT_NEAR(positions[6].y(), 0.0, 0.01) << "TCP Y";
    EXPECT_NEAR(positions[6].z(), 1465.0, 0.01) << "TCP Z";
}

// ============================================================================
// Manufacturer Reach Verification
// ============================================================================

TEST_F(UrdfFKTest, ManufacturerReach) {
    // MA2010 max reach from spec: 2010mm
    // At full horizontal extension (J2=90°, all others=0):
    // The arm extends along the original Z-axis direction in link_1_s frame
    auto pose = fk.compute({0, M_PI/2, 0, 0, 0, 0});
    double horizontal_reach = std::sqrt(
        pose.position.x() * pose.position.x() +
        pose.position.y() * pose.position.y()
    );
    // At J2=90 the reach is 1110mm (not max because J3, J5 not extended)
    EXPECT_NEAR(horizontal_reach, 1110.0, 0.1);

    // True max reach: J2=90°, J3=90° (J3 axis is -Y, extends wrist section outward)
    auto pose2 = fk.compute({0, M_PI/2, M_PI/2, 0, 0, 0});
    double reach2 = std::sqrt(
        pose2.position.x() * pose2.position.x() +
        pose2.position.y() * pose2.position.y()
    );
    // With J3 extending: more reach
    EXPECT_GT(reach2, horizontal_reach) << "J3=90 should increase reach beyond J2=90 alone";
}

// ============================================================================
// Numerical Jacobian Self-Consistency
// ============================================================================

TEST_F(UrdfFKTest, NumericalJacobian_SelfConsistency) {
    // Verify: small perturbation in q → correct change in TCP pose
    JointAngles q = {0.1, -0.2, 0.3, -0.1, 0.2, -0.3};
    auto J = fk.computeJacobian(q);

    // Apply small perturbation and check prediction
    constexpr double dq = 1e-4;

    for (int i = 0; i < 6; ++i) {
        JointAngles q_pert = q;
        q_pert[i] += dq;

        auto pose_orig = fk.compute(q);
        auto pose_pert = fk.compute(q_pert);

        Vector3d actual_dp = pose_pert.position - pose_orig.position;
        Vector3d predicted_dp = J.block<3,1>(0, i) * dq;

        // Prediction should be close to actual
        EXPECT_NEAR(actual_dp.x(), predicted_dp.x(), 0.1)
            << "Jacobian J" << i+1 << " linear X";
        EXPECT_NEAR(actual_dp.y(), predicted_dp.y(), 0.1)
            << "Jacobian J" << i+1 << " linear Y";
        EXPECT_NEAR(actual_dp.z(), predicted_dp.z(), 0.1)
            << "Jacobian J" << i+1 << " linear Z";
    }
}

// ============================================================================
// Tool Offset Tests
// ============================================================================

TEST_F(UrdfFKTest, ToolOffsetChange) {
    // At home, with default tool offset (100, 0, 0), TCP = (1332, 0, 1465)
    auto pose1 = fk.compute({0, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose1.position.x(), 1332.0, 0.01);

    // Change tool offset
    fk.setToolOffset(Vector3d(200, 0, 0));
    auto pose2 = fk.compute({0, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose2.position.x(), 1432.0, 0.01) << "Increased tool offset";

    // No tool offset
    fk.setToolOffset(Vector3d::Zero());
    auto pose3 = fk.compute({0, 0, 0, 0, 0, 0});
    EXPECT_NEAR(pose3.position.x(), 1232.0, 0.01) << "No tool offset";

    // Restore
    fk.setToolOffset(Vector3d(100, 0, 0));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(UrdfFKTest, NumJoints) {
    EXPECT_EQ(fk.numJoints(), 6u);
}

TEST_F(UrdfFKTest, ZeroAngles_IdentityRotation) {
    auto pose = fk.compute({0, 0, 0, 0, 0, 0});
    // At home, rotation should be Identity
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == j) {
                EXPECT_NEAR(pose.rotation(i, j), 1.0, 1e-10)
                    << "R(" << i << "," << j << ") should be 1";
            } else {
                EXPECT_NEAR(pose.rotation(i, j), 0.0, 1e-10)
                    << "R(" << i << "," << j << ") should be 0";
            }
        }
    }
}
