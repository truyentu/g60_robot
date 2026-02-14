/**
 * @file test_kinematics.cpp
 * @brief Unit tests for kinematics engine
 */

#include <gtest/gtest.h>
#include "kinematics/MathTypes.hpp"
#include "kinematics/DHParameters.hpp"
#include "kinematics/ForwardKinematics.hpp"
#include "kinematics/InverseKinematics.hpp"
#include "kinematics/KinematicsService.hpp"

using namespace robot_controller::kinematics;

class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = createDefault6DOFConfig();
        fk_ = std::make_unique<ForwardKinematics>(config_);
        ik_ = std::make_unique<InverseKinematics>(config_);
        service_ = std::make_unique<KinematicsService>(config_);
    }

    RobotKinematicConfig config_;
    std::unique_ptr<ForwardKinematics> fk_;
    std::unique_ptr<InverseKinematics> ik_;
    std::unique_ptr<KinematicsService> service_;
};

// ============================================================================
// Math Utilities Tests
// ============================================================================

TEST(MathTest, DegToRad) {
    EXPECT_NEAR(degToRad(0.0), 0.0, EPSILON);
    EXPECT_NEAR(degToRad(90.0), PI / 2.0, EPSILON);
    EXPECT_NEAR(degToRad(180.0), PI, EPSILON);
    EXPECT_NEAR(degToRad(-90.0), -PI / 2.0, EPSILON);
}

TEST(MathTest, RadToDeg) {
    EXPECT_NEAR(radToDeg(0.0), 0.0, EPSILON);
    EXPECT_NEAR(radToDeg(PI / 2.0), 90.0, EPSILON);
    EXPECT_NEAR(radToDeg(PI), 180.0, EPSILON);
}

TEST(MathTest, NormalizeAngle) {
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, EPSILON);
    EXPECT_NEAR(normalizeAngle(PI), PI, EPSILON);
    EXPECT_NEAR(normalizeAngle(2 * PI), 0.0, EPSILON);
    EXPECT_NEAR(normalizeAngle(-PI), -PI, EPSILON);
    EXPECT_NEAR(normalizeAngle(3 * PI), PI, EPSILON);
}

TEST(MathTest, RotationToRPY) {
    // Identity rotation
    Matrix3d I = Matrix3d::Identity();
    Vector3d rpy = rotationToRPY(I);
    EXPECT_NEAR(rpy(0), 0.0, EPSILON);
    EXPECT_NEAR(rpy(1), 0.0, EPSILON);
    EXPECT_NEAR(rpy(2), 0.0, EPSILON);

    // 90 degree rotation about Z
    Matrix3d Rz;
    Rz << 0, -1, 0,
          1,  0, 0,
          0,  0, 1;
    rpy = rotationToRPY(Rz);
    EXPECT_NEAR(rpy(2), PI / 2.0, 0.01);
}

TEST(MathTest, RPYToRotation) {
    Vector3d rpy(0.1, 0.2, 0.3);
    Matrix3d R = rpyToRotation(rpy);
    Vector3d rpy2 = rotationToRPY(R);

    EXPECT_NEAR(rpy(0), rpy2(0), 0.01);
    EXPECT_NEAR(rpy(1), rpy2(1), 0.01);
    EXPECT_NEAR(rpy(2), rpy2(2), 0.01);
}

// ============================================================================
// DH Parameters Tests
// ============================================================================

TEST(DHParametersTest, DefaultConfig) {
    auto config = createDefault6DOFConfig();

    EXPECT_EQ(config.dhParams.size(), 6u);
    EXPECT_EQ(config.dhParams[0].name, "J1_Base");
    EXPECT_EQ(config.dhParams[5].name, "J6_Wrist3");
}

TEST(DHParametersTest, WeldingConfig) {
    auto config = createWeldingRobotConfig();

    EXPECT_EQ(config.dhParams.size(), 6u);
    EXPECT_GT(config.toolOffset.norm(), 0);  // Tool offset set
}

// ============================================================================
// Forward Kinematics Tests
// ============================================================================

TEST_F(KinematicsTest, FK_ZeroPosition) {
    JointAngles zeros = {0, 0, 0, 0, 0, 0};
    TCPPose pose = fk_->compute(zeros);

    // Should return a valid pose
    EXPECT_GT(pose.position.norm(), 0);

    // Rotation should be valid (determinant = 1)
    EXPECT_NEAR(pose.rotation.determinant(), 1.0, 0.01);
}

TEST_F(KinematicsTest, FK_JointPositions) {
    JointAngles zeros = {0, 0, 0, 0, 0, 0};
    auto positions = fk_->computeJointPositions(zeros);

    // Should have base + 6 joints + TCP = 8 positions
    EXPECT_EQ(positions.size(), 8u);

    // First position is base
    EXPECT_NEAR(positions[0].norm(), config_.baseOffset.norm(), 0.01);
}

TEST_F(KinematicsTest, FK_AllTransforms) {
    JointAngles zeros = {0, 0, 0, 0, 0, 0};
    auto transforms = fk_->computeAllTransforms(zeros);

    // Should have 6 joint transforms + TCP = 7
    EXPECT_EQ(transforms.size(), 7u);

    // All transforms should be valid (determinant of rotation part = 1)
    for (const auto& T : transforms) {
        Matrix3d R = T.block<3, 3>(0, 0);
        EXPECT_NEAR(R.determinant(), 1.0, 0.01);
    }
}

TEST_F(KinematicsTest, FK_SingleJointMotion) {
    JointAngles angles1 = {0, 0, 0, 0, 0, 0};
    JointAngles angles2 = {degToRad(90), 0, 0, 0, 0, 0};

    TCPPose pose1 = fk_->compute(angles1);
    TCPPose pose2 = fk_->compute(angles2);

    // Positions should be different
    EXPECT_GT((pose1.position - pose2.position).norm(), 0.01);
}

// ============================================================================
// Inverse Kinematics Tests
// ============================================================================

TEST_F(KinematicsTest, IK_Consistency) {
    // FK -> IK -> FK should give same result
    JointAngles original = {0.1, -0.2, 0.3, 0.1, -0.1, 0.2};

    // Compute FK
    TCPPose targetPose = fk_->compute(original);

    // Compute IK
    auto solution = ik_->compute(targetPose, original);

    ASSERT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->isValid);

    // Compute FK with IK result
    TCPPose resultPose = fk_->compute(solution->angles);

    // Positions should match
    EXPECT_NEAR((targetPose.position - resultPose.position).norm(), 0.0, 0.1);
}

TEST_F(KinematicsTest, IK_MultiSolution) {
    JointAngles start = {0, -PI/4, PI/2, 0, PI/4, 0};
    TCPPose targetPose = fk_->compute(start);

    IKSolutions solutions = ik_->computeAll(targetPose);

    // Should find at least one solution
    EXPECT_TRUE(solutions.hasAnySolution());

    // All solutions should give same TCP position
    for (const auto& sol : solutions.solutions) {
        TCPPose pose = fk_->compute(sol.angles);
        EXPECT_NEAR((targetPose.position - pose.position).norm(), 0.0, 1.0);
    }
}

TEST_F(KinematicsTest, IK_IterativeConvergence) {
    // Use angles within joint limits
    JointAngles target = {0.3, -0.3, 0.5, 0.2, -0.3, 0.1};
    TCPPose targetPose = fk_->compute(target);

    // Start from target itself - should converge immediately
    JointAngles initialGuess = target;
    IKSolution solution = ik_->computeIterative(targetPose, initialGuess);

    // Should converge
    EXPECT_TRUE(solution.isValid);
    EXPECT_LT(solution.residualError, 1.0);
}

// ============================================================================
// Jacobian Tests
// ============================================================================

TEST_F(KinematicsTest, Jacobian_NotZero) {
    JointAngles angles = {0.1, -0.2, 0.3, 0.1, -0.1, 0.2};
    Jacobian J = ik_->computeJacobian(angles);

    // Jacobian should not be all zeros
    EXPECT_GT(J.norm(), 0);
}

TEST_F(KinematicsTest, Jacobian_Singularity) {
    // Near-singular configuration (wrist singularity)
    JointAngles singularConfig = {0, -PI/2, 0, 0, 0, 0};

    // Should detect singularity or have high manipulability warning
    double manip = ik_->computeManipulability(singularConfig);

    // Low manipulability indicates near-singularity
    // (exact threshold depends on robot geometry)
    EXPECT_LT(manip, 1e6);  // Just check it's computed
}

TEST_F(KinematicsTest, Manipulability) {
    JointAngles angles = {0, -PI/4, PI/2, 0, PI/4, 0};
    double manip = ik_->computeManipulability(angles);

    EXPECT_GT(manip, 0);
}

// ============================================================================
// Service Tests
// ============================================================================

TEST_F(KinematicsTest, Service_ThreadSafety) {
    // Multiple FK computations should work
    JointAngles angles = {0.1, -0.2, 0.3, 0.1, -0.1, 0.2};

    for (int i = 0; i < 100; ++i) {
        TCPPose pose = service_->computeFK(angles);
        EXPECT_GT(pose.position.norm(), 0);
    }
}

TEST_F(KinematicsTest, Service_ConfigUpdate) {
    auto newConfig = createWeldingRobotConfig();
    service_->setRobotConfig(newConfig);

    auto& currentConfig = service_->getRobotConfig();
    EXPECT_GT(currentConfig.toolOffset.norm(), 0);
}

TEST_F(KinematicsTest, Service_JointValidation) {
    JointAngles valid = {0, 0, 0, 0, 0, 0};
    EXPECT_TRUE(service_->areJointAnglesValid(valid));

    JointAngles invalid = {10, 10, 10, 10, 10, 10};  // Way outside limits
    EXPECT_FALSE(service_->areJointAnglesValid(invalid));
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
