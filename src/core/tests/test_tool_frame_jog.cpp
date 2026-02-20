/**
 * @file test_tool_frame_jog.cpp
 * @brief Unit tests for Cartesian jog in Tool frame (frame=2)
 *
 * Verifies that when jogging along one tool axis (X, Y, or Z),
 * only the expected world-frame component changes, orientation stays fixed,
 * and no sudden jumps occur.
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
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::kinematics;

// ============================================================================
// MA2010 URDF Joint Definitions
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

class ToolFrameJogTest : public ::testing::Test {
public:  // public so helper functions can access JogStep
    std::shared_ptr<firmware::FirmwareSimulator> sim_;
    std::unique_ptr<jog::JogController> jogCtrl_;
    std::vector<UrdfJointDef> joints_;

    void SetUp() override {
        Logger::init("test_tool_frame_jog.log", "debug");

        sim_ = std::make_shared<firmware::FirmwareSimulator>();
        jogCtrl_ = std::make_unique<jog::JogController>();
        jogCtrl_->setFirmwareDriver(sim_);

        joints_ = buildMA2010Joints();
        Vector3d toolOffset(100, 0, 0);
        jogCtrl_->initializeUrdfKinematics(joints_, toolOffset);

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

    void setSimJoints(const std::array<double, 6>& deg) {
        sim_->setJointPositionsDirect(deg);
        sim_->update(0.01);
    }

    TCPPose getCurrentTcpPose() {
        return jogCtrl_->getTcpPoseObject();
    }

    // Run jog for N steps and return TCP pose history
    struct JogStep {
        TCPPose pose;
        Vector3d rpyDeg;  // RPY in degrees for easier comparison
    };

    std::vector<JogStep> runJog(int axis, int direction, int numSteps,
                                 double speedPercent = 30.0, double dt = 0.004) {
        std::vector<JogStep> history;

        // Record start pose
        JogStep start;
        start.pose = getCurrentTcpPose();
        start.rpyDeg = start.pose.rpy * 180.0 / M_PI;
        history.push_back(start);

        // frame=2 = Tool frame
        auto err = jogCtrl_->startContinuousJog(1, axis, direction, speedPercent, 2);
        EXPECT_EQ(err, "") << "startContinuousJog failed: " << err;

        for (int i = 0; i < numSteps; i++) {
            jogCtrl_->update(dt);

            JogStep step;
            step.pose = getCurrentTcpPose();
            step.rpyDeg = step.pose.rpy * 180.0 / M_PI;
            history.push_back(step);
        }

        jogCtrl_->stopJog();
        return history;
    }
};

// ============================================================================
// Test configs: Different orientations to test tool frame jog
// ============================================================================

// Config 1: Home-like position (tool pointing along world X)
static const std::array<double, 6> CONFIG_HOME = {0.0, 30.0, -30.0, 0.0, -45.0, 0.0};

// Config 2: Robot rotated 90° in J1 (tool pointing along world Y)
static const std::array<double, 6> CONFIG_ROTATED = {90.0, 30.0, -30.0, 0.0, -45.0, 0.0};

// Config 3: Wrist rotated (mixed orientation)
static const std::array<double, 6> CONFIG_WRIST = {45.0, 20.0, -20.0, 30.0, -60.0, 15.0};

// ============================================================================
// Helper: Check that orientation stays constant during jog
// ============================================================================
static void expectOrientationStable(const std::vector<ToolFrameJogTest::JogStep>& history,
                                     double maxRpyChangeDeg = 0.5) {
    const auto& startRpy = history[0].rpyDeg;
    for (size_t i = 1; i < history.size(); i++) {
        const auto& rpy = history[i].rpyDeg;
        for (int a = 0; a < 3; a++) {
            double diff = std::abs(rpy[a] - startRpy[a]);
            EXPECT_LT(diff, maxRpyChangeDeg)
                << "RPY axis " << a << " changed by " << diff
                << "° at step " << i
                << " (start=" << startRpy[a] << " now=" << rpy[a] << ")";
        }
    }
}

// ============================================================================
// Helper: Check no sudden position jumps between consecutive steps
// ============================================================================
static void expectNoPositionJumps(const std::vector<ToolFrameJogTest::JogStep>& history,
                                   double maxJumpMm = 5.0) {
    for (size_t i = 1; i < history.size(); i++) {
        double jump = (history[i].pose.position - history[i-1].pose.position).norm();
        EXPECT_LT(jump, maxJumpMm)
            << "Position jump of " << jump << "mm at step " << i
            << " from " << history[i-1].pose.position.transpose()
            << " to " << history[i].pose.position.transpose();
    }
}

// ============================================================================
// Helper: Check that tool-frame movement is along expected axis
// ============================================================================
static void expectToolAxisMovement(const std::vector<ToolFrameJogTest::JogStep>& history,
                                    int toolAxis, int direction,
                                    double minTotalMm = 1.0) {
    const auto& startPose = history[0].pose;
    const auto& endPose = history.back().pose;

    // Get tool frame axes from start rotation
    Matrix3d R = startPose.rotation;
    Vector3d toolX = R.col(0);  // Tool X axis in world
    Vector3d toolY = R.col(1);  // Tool Y axis in world
    Vector3d toolZ = R.col(2);  // Tool Z axis in world

    Vector3d totalDisplacement = endPose.position - startPose.position;
    double totalDist = totalDisplacement.norm();

    // Project displacement onto tool axes
    double projX = totalDisplacement.dot(toolX);
    double projY = totalDisplacement.dot(toolY);
    double projZ = totalDisplacement.dot(toolZ);

    std::cout << "[TOOL JOG] axis=" << toolAxis << " dir=" << direction
              << " totalDist=" << totalDist << "mm"
              << " projX=" << projX << " projY=" << projY << " projZ=" << projZ
              << std::endl;

    // The commanded axis should have significant movement
    double projOnAxis = (toolAxis == 0) ? projX : (toolAxis == 1) ? projY : projZ;
    EXPECT_GT(std::abs(projOnAxis), minTotalMm)
        << "Expected movement along tool axis " << toolAxis
        << " but only " << projOnAxis << "mm";

    // Direction should match
    if (direction > 0) {
        EXPECT_GT(projOnAxis, 0.0) << "Direction should be positive";
    } else {
        EXPECT_LT(projOnAxis, 0.0) << "Direction should be negative";
    }

    // Cross-axis leakage should be small relative to main axis movement
    double crossAxisThreshold = std::abs(projOnAxis) * 0.1;  // Max 10% leakage
    if (toolAxis != 0) {
        EXPECT_LT(std::abs(projX), std::max(crossAxisThreshold, 1.0))
            << "Unexpected X-axis leakage: " << projX << "mm";
    }
    if (toolAxis != 1) {
        EXPECT_LT(std::abs(projY), std::max(crossAxisThreshold, 1.0))
            << "Unexpected Y-axis leakage: " << projY << "mm";
    }
    if (toolAxis != 2) {
        EXPECT_LT(std::abs(projZ), std::max(crossAxisThreshold, 1.0))
            << "Unexpected Z-axis leakage: " << projZ << "mm";
    }
}

// ============================================================================
// TEST: Tool frame Jog Z+ from home config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolZ_Positive_HomeConfig) {
    setSimJoints(CONFIG_HOME);
    auto history = runJog(2, 1, 200);  // axis=Z, dir=+1

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 2, 1);
}

// ============================================================================
// TEST: Tool frame Jog Z- from home config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolZ_Negative_HomeConfig) {
    setSimJoints(CONFIG_HOME);
    auto history = runJog(2, -1, 200);  // axis=Z, dir=-1

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 2, -1);
}

// ============================================================================
// TEST: Tool frame Jog X+ from home config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolX_Positive_HomeConfig) {
    setSimJoints(CONFIG_HOME);
    auto history = runJog(0, 1, 200);  // axis=X, dir=+1

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 0, 1);
}

// ============================================================================
// TEST: Tool frame Jog Y+ from home config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolY_Positive_HomeConfig) {
    setSimJoints(CONFIG_HOME);
    auto history = runJog(1, 1, 200);  // axis=Y, dir=+1

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 1, 1);
}

// ============================================================================
// TEST: Tool frame Jog Z+ from rotated config (J1=90°)
// ============================================================================
TEST_F(ToolFrameJogTest, ToolZ_Positive_RotatedConfig) {
    setSimJoints(CONFIG_ROTATED);
    auto history = runJog(2, 1, 200);  // axis=Z, dir=+1

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 2, 1);
}

// ============================================================================
// TEST: Tool frame Jog Z- from rotated config (J1=90°)
// ============================================================================
TEST_F(ToolFrameJogTest, ToolZ_Negative_RotatedConfig) {
    setSimJoints(CONFIG_ROTATED);
    auto history = runJog(2, -1, 200);

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 2, -1);
}

// ============================================================================
// TEST: Tool frame Jog X+ from wrist-rotated config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolX_Positive_WristConfig) {
    setSimJoints(CONFIG_WRIST);
    auto history = runJog(0, 1, 200);

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 0, 1);
}

// ============================================================================
// TEST: Tool frame Jog Y- from wrist-rotated config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolY_Negative_WristConfig) {
    setSimJoints(CONFIG_WRIST);
    auto history = runJog(1, -1, 200);

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 1, -1);
}

// ============================================================================
// TEST: Tool frame Jog Z+ from wrist-rotated config
// ============================================================================
TEST_F(ToolFrameJogTest, ToolZ_Positive_WristConfig) {
    setSimJoints(CONFIG_WRIST);
    auto history = runJog(2, 1, 200);

    expectNoPositionJumps(history);
    expectOrientationStable(history);
    expectToolAxisMovement(history, 2, 1);
}

// ============================================================================
// TEST: All 3 axes sequentially from same start (verify independence)
// ============================================================================
TEST_F(ToolFrameJogTest, AllAxes_Independence) {
    for (int axis = 0; axis < 3; axis++) {
        // Reset to same starting config each time
        setSimJoints(CONFIG_HOME);

        auto history = runJog(axis, 1, 150, 30.0);

        SCOPED_TRACE("Tool axis " + std::to_string(axis));
        expectNoPositionJumps(history);
        expectOrientationStable(history);
        expectToolAxisMovement(history, axis, 1);
    }
}
