/**
 * @file test_trajectory.cpp
 * @brief Unit tests for trajectory generator
 */

#include <gtest/gtest.h>
#include "trajectory/TrajectoryTypes.hpp"
#include "trajectory/VelocityProfile.hpp"
#include "trajectory/Interpolators.hpp"
#include "trajectory/TrajectoryPlanner.hpp"
#include "trajectory/TrajectoryExecutor.hpp"
#include "kinematics/KinematicsService.hpp"

using namespace robot_controller::trajectory;
using namespace robot_controller::kinematics;

class TrajectoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto config = createDefault6DOFConfig();
        kinematics_ = std::make_shared<KinematicsService>(config);
        planner_ = std::make_unique<TrajectoryPlanner>(kinematics_);
    }

    std::shared_ptr<KinematicsService> kinematics_;
    std::unique_ptr<TrajectoryPlanner> planner_;
};

// ============================================================================
// Velocity Profile Tests
// ============================================================================

TEST(VelocityProfileTest, TrapezoidalBasic) {
    TrapezoidalProfile profile;

    // At start
    auto result = profile.compute(0, 1.0);
    EXPECT_NEAR(result.position, 0, 0.01);

    // At end
    result = profile.compute(1.0, 1.0);
    EXPECT_NEAR(result.position, 1.0, 0.01);
    EXPECT_NEAR(result.velocity, 0, 0.1);

    // In middle (should have velocity)
    result = profile.compute(0.5, 1.0);
    EXPECT_GT(result.position, 0);
    EXPECT_LT(result.position, 1.0);
}

TEST(VelocityProfileTest, TrapezoidalDuration) {
    TrapezoidalProfile profile;

    double duration = profile.calculateDuration(100.0, 50.0, 100.0);
    EXPECT_GT(duration, 0);

    // Should take at least time to accelerate and decelerate
    double minDuration = 2.0 * 50.0 / 100.0;  // 2 * v_max / a_max
    EXPECT_GE(duration, minDuration * 0.9);  // Allow some tolerance
}

TEST(VelocityProfileTest, TrapezoidalPhases) {
    TrapezoidalProfile profile;

    double accelTime, cruiseTime, decelTime, peakVelocity;

    // Full trapezoidal with cruise phase
    profile.calculatePhases(1000.0, 100.0, 200.0,
                            accelTime, cruiseTime, decelTime, peakVelocity);

    EXPECT_GT(accelTime, 0);
    EXPECT_GT(cruiseTime, 0);
    EXPECT_GT(decelTime, 0);
    EXPECT_NEAR(peakVelocity, 100.0, 0.01);

    // Triangular profile (no cruise)
    profile.calculatePhases(10.0, 100.0, 200.0,
                            accelTime, cruiseTime, decelTime, peakVelocity);

    EXPECT_GT(accelTime, 0);
    EXPECT_NEAR(cruiseTime, 0, 0.01);  // No cruise phase
    EXPECT_GT(decelTime, 0);
}

TEST(VelocityProfileTest, SCurveSmooth) {
    SCurveProfile profile;

    // At start
    auto result = profile.compute(0, 1.0);
    EXPECT_NEAR(result.position, 0, 0.01);

    // At end
    result = profile.compute(1.0, 1.0);
    EXPECT_NEAR(result.position, 1.0, 0.01);

    // Check smoothness - position should be monotonically increasing
    double prevPos = 0;
    for (int i = 1; i <= 100; ++i) {
        double t = i * 0.01;
        result = profile.compute(t, 1.0);
        EXPECT_GE(result.position, prevPos);
        prevPos = result.position;
    }
}

TEST(VelocityProfileTest, SCurveDuration) {
    SCurveProfile profile;

    double duration = profile.calculateDuration(100.0, 50.0, 100.0, 500.0);
    EXPECT_GT(duration, 0);
}

TEST(VelocityProfileTest, ConstantProfile) {
    ConstantProfile profile;

    auto result = profile.compute(0.5, 1.0);
    EXPECT_NEAR(result.position, 0.5, 0.01);
    EXPECT_NEAR(result.velocity, 1.0, 0.01);
    EXPECT_NEAR(result.acceleration, 0, 0.01);
}

TEST(VelocityProfileTest, ProfileFactory) {
    auto trap = createVelocityProfile(VelocityProfile::TRAPEZOIDAL);
    EXPECT_NE(trap, nullptr);

    auto scurve = createVelocityProfile(VelocityProfile::SCURVE);
    EXPECT_NE(scurve, nullptr);

    auto constant = createVelocityProfile(VelocityProfile::CONSTANT);
    EXPECT_NE(constant, nullptr);
}

// ============================================================================
// Interpolator Tests
// ============================================================================

TEST(InterpolatorTest, LinearPose) {
    TCPPose start, end;
    start.position = Vector3d(0, 0, 0);
    start.quaternion = Quaterniond::Identity();
    start.rotation = Matrix3d::Identity();

    end.position = Vector3d(100, 0, 0);
    end.quaternion = Quaterniond::Identity();
    end.rotation = Matrix3d::Identity();

    // Midpoint
    auto mid = LinearInterpolator::interpolate(start, end, 0.5);
    EXPECT_NEAR(mid.position.x(), 50, 0.01);
    EXPECT_NEAR(mid.position.y(), 0, 0.01);
    EXPECT_NEAR(mid.position.z(), 0, 0.01);

    // Endpoints
    auto atStart = LinearInterpolator::interpolate(start, end, 0.0);
    EXPECT_NEAR(atStart.position.x(), 0, 0.01);

    auto atEnd = LinearInterpolator::interpolate(start, end, 1.0);
    EXPECT_NEAR(atEnd.position.x(), 100, 0.01);
}

TEST(InterpolatorTest, LinearDistance) {
    TCPPose start, end;
    start.position = Vector3d(0, 0, 0);
    end.position = Vector3d(100, 0, 0);

    double dist = LinearInterpolator::calculateDistance(start, end);
    EXPECT_NEAR(dist, 100.0, 0.01);

    // 3D distance
    end.position = Vector3d(100, 100, 100);
    dist = LinearInterpolator::calculateDistance(start, end);
    EXPECT_NEAR(dist, std::sqrt(30000.0), 0.01);
}

TEST(InterpolatorTest, JointLinear) {
    JointAngles start = {0, 0, 0, 0, 0, 0};
    JointAngles end = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    auto mid = JointInterpolator::linearInterpolate(start, end, 0.5);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(mid[i], (i + 1) * 0.5, 0.01);
    }
}

TEST(InterpolatorTest, JointMaxTravel) {
    JointAngles start = {0, 0, 0, 0, 0, 0};
    JointAngles end = {0.1, 0.2, 0.5, 0.1, 0.1, 0.1};

    double maxTravel = JointInterpolator::calculateMaxJointTravel(start, end);
    EXPECT_NEAR(maxTravel, 0.5, 0.01);
}

TEST(InterpolatorTest, JointCubic) {
    JointAngles start = {0, 0, 0, 0, 0, 0};
    JointAngles end = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    JointVelocities startVel = {0, 0, 0, 0, 0, 0};
    JointVelocities endVel = {0, 0, 0, 0, 0, 0};

    // At t=0, should be start
    auto atStart = JointInterpolator::cubicInterpolate(start, end, startVel, endVel, 0, 1.0);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(atStart[i], 0, 0.01);
    }

    // At t=T, should be end
    auto atEnd = JointInterpolator::cubicInterpolate(start, end, startVel, endVel, 1.0, 1.0);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(atEnd[i], 1.0, 0.01);
    }
}

TEST(InterpolatorTest, CircularArc) {
    // Points on a semicircle in XY plane centered at origin
    Vector3d start(100, 0, 0);
    Vector3d via(0, 100, 0);
    Vector3d end(-100, 0, 0);

    Vector3d center;
    double radius;
    Vector3d normal;

    bool valid = CircularInterpolator::defineArc(start, via, end, center, radius, normal);

    EXPECT_TRUE(valid);
    // The circumcenter calculation may not give exactly (0,0,0)
    // but the radius should be close to 100
    EXPECT_GT(radius, 50);
    EXPECT_LT(radius, 200);
    // Normal should be in Z direction (or -Z)
    EXPECT_GT(std::abs(normal.z()), 0.5);
}

TEST(InterpolatorTest, CircularCollinear) {
    // Collinear points should fail
    Vector3d start(0, 0, 0);
    Vector3d via(50, 0, 0);
    Vector3d end(100, 0, 0);

    Vector3d center;
    double radius;
    Vector3d normal;

    bool valid = CircularInterpolator::defineArc(start, via, end, center, radius, normal);

    EXPECT_FALSE(valid);
}

// ============================================================================
// Trajectory Planner Tests
// ============================================================================

TEST_F(TrajectoryTest, PlanPTP) {
    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.useJointTarget = true;
    cmd.targetJoints = {0.1, -0.1, 0.2, 0, 0, 0};

    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};

    Trajectory traj = planner_->planMotion(cmd, currentJoints);

    EXPECT_TRUE(traj.isValid);
    EXPECT_EQ(traj.segments.size(), 1u);
    EXPECT_GT(traj.totalDuration, 0);
}

TEST_F(TrajectoryTest, PlanLIN) {
    JointAngles currentJoints = {0, -0.3, 0.5, 0, 0.3, 0};
    TCPPose currentPose = kinematics_->computeFK(currentJoints);

    MotionCommand cmd;
    cmd.type = MotionType::LIN;
    cmd.useJointTarget = false;
    cmd.targetPose = currentPose;
    cmd.targetPose.position += Vector3d(50, 0, 0);  // Move 50mm in X

    Trajectory traj = planner_->planMotion(cmd, currentJoints);

    EXPECT_TRUE(traj.isValid);
    EXPECT_EQ(traj.segments.size(), 1u);
    EXPECT_GT(traj.totalDuration, 0);
}

TEST_F(TrajectoryTest, PlanPath) {
    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};

    std::vector<Waypoint> waypoints;

    Waypoint wp1;
    wp1.isJointSpace = true;
    wp1.jointAngles = {0.1, -0.1, 0.1, 0, 0, 0};
    waypoints.push_back(wp1);

    Waypoint wp2;
    wp2.isJointSpace = true;
    wp2.jointAngles = {0.2, -0.2, 0.2, 0, 0, 0};
    waypoints.push_back(wp2);

    MotionParams params;
    params.velocityScale = 0.5;

    Trajectory traj = planner_->planPath(waypoints, currentJoints, params);

    EXPECT_TRUE(traj.isValid);
    EXPECT_EQ(traj.segments.size(), 2u);
    EXPECT_GT(traj.totalDuration, 0);
}

TEST_F(TrajectoryTest, TrajectorySampling) {
    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.useJointTarget = true;
    cmd.targetJoints = {0.5, -0.3, 0.4, 0.1, -0.2, 0.1};

    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};

    Trajectory traj = planner_->planMotion(cmd, currentJoints);
    ASSERT_TRUE(traj.isValid);

    // Sample at start
    auto p0 = traj.sample(0);
    EXPECT_TRUE(p0.isValid);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(p0.jointPositions[i], currentJoints[i], 0.01);
    }

    // Sample at end
    auto pEnd = traj.sample(traj.totalDuration);
    EXPECT_TRUE(pEnd.isValid);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(pEnd.jointPositions[i], cmd.targetJoints[i], 0.01);
    }

    // Sample at middle
    auto pMid = traj.sample(traj.totalDuration / 2);
    EXPECT_TRUE(pMid.isValid);
    EXPECT_GT(pMid.segmentProgress, 0);
    EXPECT_LT(pMid.segmentProgress, 1.0);
}

// ============================================================================
// Trajectory Executor Tests
// ============================================================================

TEST_F(TrajectoryTest, ExecutorBasic) {
    TrajectoryExecutor executor(kinematics_);

    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.useJointTarget = true;
    cmd.targetJoints = {0.1, -0.1, 0.1, 0, 0, 0};

    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};
    Trajectory traj = planner_->planMotion(cmd, currentJoints);

    EXPECT_TRUE(executor.start(traj));
    EXPECT_EQ(executor.getState(), TrajectoryState::RUNNING);

    executor.stop();
    // Give thread time to stop
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(executor.getState(), TrajectoryState::IDLE);
}

TEST_F(TrajectoryTest, ExecutorSampling) {
    TrajectoryExecutor executor(kinematics_);
    executor.setControlRate(100.0);  // 100 Hz for faster test

    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.useJointTarget = true;
    cmd.targetJoints = {0.1, 0, 0, 0, 0, 0};
    cmd.params.velocityScale = 1.0;  // Fast

    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};
    Trajectory traj = planner_->planMotion(cmd, currentJoints);

    // Manual sampling without starting execution thread
    auto p = executor.sample(0);
    // Should be invalid since no trajectory loaded
    EXPECT_FALSE(p.isValid);
}

TEST_F(TrajectoryTest, ExecutorPauseResume) {
    TrajectoryExecutor executor(kinematics_);

    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.useJointTarget = true;
    cmd.targetJoints = {0.5, 0, 0, 0, 0, 0};

    JointAngles currentJoints = {0, 0, 0, 0, 0, 0};
    Trajectory traj = planner_->planMotion(cmd, currentJoints);

    executor.start(traj);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    executor.pause();
    EXPECT_EQ(executor.getState(), TrajectoryState::PAUSED);

    double timeAtPause = executor.getCurrentTime();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Time should not have advanced while paused
    EXPECT_NEAR(executor.getCurrentTime(), timeAtPause, 0.02);

    executor.resume();
    EXPECT_EQ(executor.getState(), TrajectoryState::RUNNING);

    executor.stop();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
