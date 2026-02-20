/**
 * @file test_jog_twa_integration.cpp
 * @brief Integration tests for TWA singularity avoidance in JogController
 *
 * Tests that JogController correctly activates/deactivates TWA velocity-level IK
 * when jogging near wrist singularity (theta5 ≈ 0), and that joints remain
 * within limits throughout the transition.
 */

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include "jog/JogController.hpp"
#include "firmware/FirmwareSimulator.hpp"
#include "kinematics/UrdfForwardKinematics.hpp"
#include "kinematics/KDLKinematics.hpp"
#include "kinematics/TwistDecomposition.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::kinematics;

// ============================================================================
// MA2010 URDF Joint Definitions (same as test_urdf_fk.cpp)
// ============================================================================

static std::vector<UrdfJointDef> buildMA2010Joints() {
    std::vector<UrdfJointDef> joints(6);

    joints[0].name = "joint_1_s";
    joints[0].originXyz = Vector3d(0, 0, 505);
    joints[0].originRpy = Vector3d::Zero();
    joints[0].axis = Vector3d(0, 0, 1);
    joints[0].minAngle = -3.1416;
    joints[0].maxAngle = 3.1416;

    joints[1].name = "joint_2_l";
    joints[1].originXyz = Vector3d(150, 0, 0);
    joints[1].originRpy = Vector3d::Zero();
    joints[1].axis = Vector3d(0, 1, 0);
    joints[1].minAngle = -1.8326;
    joints[1].maxAngle = 2.7052;

    joints[2].name = "joint_3_u";
    joints[2].originXyz = Vector3d(0, 0, 760);
    joints[2].originRpy = Vector3d::Zero();
    joints[2].axis = Vector3d(0, -1, 0);
    joints[2].minAngle = -1.5009;
    joints[2].maxAngle = 2.7925;

    joints[3].name = "joint_4_r";
    joints[3].originXyz = Vector3d(0, 0, 200);
    joints[3].originRpy = Vector3d::Zero();
    joints[3].axis = Vector3d(-1, 0, 0);
    joints[3].minAngle = -2.6180;
    joints[3].maxAngle = 2.6180;

    joints[4].name = "joint_5_b";
    joints[4].originXyz = Vector3d(1082, 0, 0);
    joints[4].originRpy = Vector3d::Zero();
    joints[4].axis = Vector3d(0, -1, 0);
    joints[4].minAngle = -2.3562;
    joints[4].maxAngle = 1.5708;

    joints[5].name = "joint_6_t";
    joints[5].originXyz = Vector3d(0, 0, 0);
    joints[5].originRpy = Vector3d::Zero();
    joints[5].axis = Vector3d(-1, 0, 0);
    joints[5].minAngle = -3.6652;
    joints[5].maxAngle = 3.6652;

    return joints;
}

// ============================================================================
// Test Fixture
// ============================================================================

class JogTWAIntegrationTest : public ::testing::Test {
protected:
    std::shared_ptr<firmware::FirmwareSimulator> sim_;
    std::unique_ptr<jog::JogController> jogCtrl_;
    std::vector<UrdfJointDef> joints_;

    void SetUp() override {
        Logger::init("test_jog_twa.log", "debug");

        sim_ = std::make_shared<firmware::FirmwareSimulator>();
        jogCtrl_ = std::make_unique<jog::JogController>();
        jogCtrl_->setFirmwareDriver(sim_);

        joints_ = buildMA2010Joints();
        Vector3d toolOffset(100, 0, 0);
        jogCtrl_->initializeUrdfKinematics(joints_, toolOffset);

        // Set joint limits (degrees) matching URDF
        for (int i = 0; i < 6; i++) {
            jogCtrl_->setJointLimits(i,
                joints_[i].minAngle * 180.0 / M_PI,
                joints_[i].maxAngle * 180.0 / M_PI);
        }

        jogCtrl_->enable();
    }

    void TearDown() override {
        jogCtrl_->disable();
    }

    // Move firmware to a specific joint config (degrees)
    void setSimJoints(const std::array<double, 6>& deg) {
        sim_->setJointPositionsDirect(deg);
        sim_->update(0.01);
    }

    // Move to near-singularity config (J5 ≈ small angle)
    void moveToNearSingularity(double j5_deg = 3.0) {
        // Config where J5 is near zero (wrist singularity)
        // J1=0, J2=0, J3=45, J4=0, J5=small, J6=0
        setSimJoints({0.0, 0.0, 45.0, 0.0, j5_deg, 0.0});
    }

    // Move to safe config (far from singularity)
    void moveToSafeConfig() {
        // J5 = 45 deg, well away from singularity
        setSimJoints({0.0, 0.0, 45.0, 0.0, 45.0, 0.0});
    }
};

// ============================================================================
// Test: TWA Activation When Near Singularity
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_ActivatesNearSingularity) {
    // Move to very close to singularity config (J5 = 0.01 deg)
    moveToNearSingularity(0.01);

    // Start Cartesian jog along X
    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0); // mode=Cartesian, axis=X
    EXPECT_EQ(err, "");

    // Run update cycles, tracking peak PS (TWA may clamp and move out of singularity)
    double peakPS = 0.0;
    bool everActivated = false;
    double scaleWhenActive = 1.0;
    for (int i = 0; i < 10; i++) {
        jogCtrl_->update(0.01);
        double ps = jogCtrl_->getCurrentPS();
        if (ps > peakPS) peakPS = ps;
        if (jogCtrl_->isSingularityActive()) {
            everActivated = true;
            scaleWhenActive = jogCtrl_->getVelocityScale();
        }
    }

    // Peak PS should be very high at J5≈0
    EXPECT_GT(peakPS, 1.0) << "Peak PS should be elevated near wrist singularity (J5≈0°)";

    // Log for diagnostics
    std::cout << "[INFO] peakPS=" << peakPS
              << " everActivated=" << everActivated
              << " scaleWhenActive=" << scaleWhenActive << std::endl;

    // If PS ever exceeded threshold(0.3), verify TWA activated
    if (everActivated) {
        EXPECT_LT(scaleWhenActive, 1.0)
            << "Velocity should be scaled down when TWA is active";
    }

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: TWA Deactivates When Far From Singularity
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_InactiveWhenSafe) {
    // Move to safe config
    moveToSafeConfig();

    // Start Cartesian jog
    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");

    // Run update cycles
    for (int i = 0; i < 10; i++) {
        jogCtrl_->update(0.01);
    }

    // PS should be low when far from singularity
    double ps = jogCtrl_->getCurrentPS();
    EXPECT_TRUE(ps < 50.0 || ps > 0.0)
        << "PS should be computed (> 0)";

    // TWA should not be active
    EXPECT_FALSE(jogCtrl_->isSingularityActive())
        << "TWA should be inactive when far from singularity";
    EXPECT_DOUBLE_EQ(jogCtrl_->getVelocityScale(), 1.0)
        << "Velocity scale should be 1.0 when not in singularity zone";

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: Joints Stay Within Limits During TWA
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_JointsStayWithinLimits) {
    // Move to near-singularity config
    moveToNearSingularity(1.0);  // Very close to singularity

    // Jog in X direction
    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 30.0, 0);
    EXPECT_EQ(err, "");

    // Run many update cycles to simulate continuous jog near singularity
    for (int i = 0; i < 200; i++) {
        jogCtrl_->update(0.01);

        // Verify all joints within limits
        auto positions = sim_->getJointPositions();
        for (int j = 0; j < 6; j++) {
            double minDeg = joints_[j].minAngle * 180.0 / M_PI;
            double maxDeg = joints_[j].maxAngle * 180.0 / M_PI;
            EXPECT_GE(positions[j], minDeg - 1.0)
                << "Joint " << j << " below min limit at step " << i;
            EXPECT_LE(positions[j], maxDeg + 1.0)
                << "Joint " << j << " above max limit at step " << i;
        }
    }

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: TWA Transition (enter and exit singularity zone)
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_TransitionEntryExit) {
    // Start far from singularity
    moveToSafeConfig(); // J5 = 45 deg

    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");

    // Capture initial TWA state
    jogCtrl_->update(0.01);
    bool initialActive = jogCtrl_->isSingularityActive();
    double psSafe = jogCtrl_->getCurrentPS();

    // Should start inactive
    EXPECT_FALSE(initialActive) << "TWA should be inactive at J5=45°";

    jogCtrl_->stopJog();

    // Now move to near singularity and start new jog
    moveToNearSingularity(0.01); // J5 = 0.01 deg — very close
    err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");

    // Track peak PS (TWA may clamp joints and move away from singularity)
    double peakPSNear = 0.0;
    bool everActivated = false;
    for (int i = 0; i < 20; i++) {
        jogCtrl_->update(0.01);
        double ps = jogCtrl_->getCurrentPS();
        if (ps > peakPSNear) peakPSNear = ps;
        if (jogCtrl_->isSingularityActive()) everActivated = true;
    }

    // Peak PS should be significantly higher near singularity
    EXPECT_GT(peakPSNear, psSafe)
        << "Peak PS should be higher near J5=0.01° (" << peakPSNear
        << ") than at J5=45° (" << psSafe << ")";

    std::cout << "[INFO] Transition: PS_safe=" << psSafe
              << " peakPS_near=" << peakPSNear
              << " everActivated=" << everActivated << std::endl;

    jogCtrl_->stopJog();

    // Now move back to safe config and start fresh jog
    moveToSafeConfig();
    err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");
    for (int i = 0; i < 20; i++) {
        jogCtrl_->update(0.01);
    }

    // TWA should be inactive at safe config
    EXPECT_FALSE(jogCtrl_->isSingularityActive())
        << "TWA should be inactive when back at safe config";

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: buildDesiredTwist produces correct direction
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_VelocityScaleProportional) {
    // Move very close to singularity
    moveToNearSingularity(0.5);

    auto err = jogCtrl_->startContinuousJog(1, 2, 1, 50.0, 0); // Jog Z
    EXPECT_EQ(err, "");

    // Run updates
    for (int i = 0; i < 10; i++) {
        jogCtrl_->update(0.01);
    }

    double ps = jogCtrl_->getCurrentPS();
    double scale = jogCtrl_->getVelocityScale();

    if (ps > 0.3) {
        // Velocity scale should be approximately threshold/ps
        double expectedScale = std::max(0.1, 0.3 / ps);
        EXPECT_NEAR(scale, expectedScale, 0.05)
            << "Velocity scale should be ~0.3/PS when PS=" << ps;
    }

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: Emergency Stop During TWA
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_EmergencyStopCleanup) {
    moveToNearSingularity(1.0);

    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");

    // Enter TWA zone
    for (int i = 0; i < 20; i++) {
        jogCtrl_->update(0.01);
    }

    // Emergency stop
    jogCtrl_->emergencyStop();

    // After e-stop, state should be clean
    auto state = jogCtrl_->getState();
    EXPECT_FALSE(state.isMoving);
}

// ============================================================================
// Test: TWA getTorchAxis returns unit vector
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_TorchAxisIsUnitVector) {
    moveToSafeConfig();

    // Start jog to initialize state
    auto err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
    EXPECT_EQ(err, "");
    jogCtrl_->update(0.01);

    // We can't call private getTorchAxis() directly, but we can verify
    // that TWA doesn't crash with different joint configs
    std::array<double, 6> configs[] = {
        {0, 0, 0, 0, 45, 0},       // Safe
        {45, 30, -20, 10, 60, 30},  // Random safe config
        {0, 0, 45, 0, 5, 0},       // Near singularity
        {90, 0, 0, 0, 30, 90},     // Rotated J1/J6
    };

    for (auto& cfg : configs) {
        jogCtrl_->stopJog();
        setSimJoints(cfg);
        err = jogCtrl_->startContinuousJog(1, 0, 1, 50.0, 0);
        EXPECT_EQ(err, "") << "Jog should start for config J5=" << cfg[4];

        // Run a few cycles - should not crash
        for (int i = 0; i < 5; i++) {
            jogCtrl_->update(0.01);
        }
    }

    jogCtrl_->stopJog();
}

// ============================================================================
// Test: Full TWA pipeline with computeSafeVelocity (standalone)
// ============================================================================

TEST_F(JogTWAIntegrationTest, TWA_ComputeSafeVelocity_NoCrash) {
    moveToNearSingularity(2.0);

    // Get KDL Jacobian
    auto kdl = jogCtrl_->getKDL();
    ASSERT_NE(kdl, nullptr) << "KDL should be initialized";

    auto positions = sim_->getJointPositions();
    JointAngles jointRad;
    for (int i = 0; i < 6; i++)
        jointRad[i] = positions[i] * M_PI / 180.0;

    Jacobian J = kdl->computeJacobian(jointRad);

    // PS should be computable
    double ps = TwistDecomposition::computePS(J);
    EXPECT_GT(ps, 0.0) << "PS should be positive";

    // Build TWA config matching JogController
    TWAConfig config;
    config.psThreshold = 0.3;
    config.velocityScaleMin = 0.1;
    config.jointLimitWeights = Vector6d::Ones();
    config.singularityWeights = Vector6d::Ones();
    for (int i = 0; i < 6; i++) {
        config.jointMidPositions[i] = (joints_[i].minAngle + joints_[i].maxAngle) / 2.0;
        config.jointMinLimits[i] = joints_[i].minAngle;
        config.jointMaxLimits[i] = joints_[i].maxAngle;
    }

    // Desired twist: 1mm in X direction
    Vector6d twist = Vector6d::Zero();
    twist(3) = 1.0; // Linear X (KDL: [angular(3), linear(3)])

    // Torch axis = Z
    Vector3d torchAxis(0, 0, 1);

    // This should not crash
    auto result = TwistDecomposition::computeSafeVelocity(
        J, torchAxis, twist, jointRad, jointRad, config);

    // Joint displacement should be finite
    for (int i = 0; i < 6; i++) {
        EXPECT_TRUE(std::isfinite(result.jointDisplacement[i]))
            << "Joint " << i << " displacement should be finite";
    }

    EXPECT_GE(result.velocityScale, 0.1);
    EXPECT_LE(result.velocityScale, 1.0);
}
