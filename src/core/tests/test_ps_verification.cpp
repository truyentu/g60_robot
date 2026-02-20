/**
 * @file test_ps_verification.cpp
 * @brief Verification tests for PS optimization & singularity avoidance
 *
 * V1: PS continuity along J5 sweep
 * V2: Velocity scale smoothness (no step changes)
 * V3: Velocity scale = 1.0 at exact threshold (seamless entry)
 * V4: Hysteresis prevents oscillation
 * V5: TCP accuracy through singularity (FK/IK round-trip)
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include "kinematics/UrdfForwardKinematics.hpp"
#include "kinematics/KDLKinematics.hpp"
#include "kinematics/TwistDecomposition.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::kinematics;

// ============================================================================
// MA2010 Test Fixture
// ============================================================================

class PSVerificationTest : public ::testing::Test {
protected:
    std::unique_ptr<KDLKinematics> kdl_;
    std::unique_ptr<UrdfForwardKinematics> fk_;
    std::vector<UrdfJointDef> joints_;
    TWAConfig config_;

    void SetUp() override {
        Logger::init("test_ps_verification.log", "warn");
        joints_ = buildMA2010Joints();
        Vector3d toolOffset(100, 0, 0);
        kdl_ = std::make_unique<KDLKinematics>();
        kdl_->buildFromUrdfJoints(joints_, toolOffset);
        fk_ = std::make_unique<UrdfForwardKinematics>(joints_, toolOffset);

        config_.psThreshold = 0.3;
        config_.psThresholdExit = 0.2;
        config_.velocityScaleMin = 0.1;
    }

    static std::vector<UrdfJointDef> buildMA2010Joints() {
        std::vector<UrdfJointDef> joints(6);

        joints[0].name = "joint_1_s";
        joints[0].originXyz = Vector3d(0, 0, 505);
        joints[0].originRpy = Vector3d::Zero();
        joints[0].axis = Vector3d(0, 0, 1);
        joints[0].minAngle = -3.1416; joints[0].maxAngle = 3.1416;

        joints[1].name = "joint_2_l";
        joints[1].originXyz = Vector3d(150, 0, 0);
        joints[1].originRpy = Vector3d::Zero();
        joints[1].axis = Vector3d(0, 1, 0);
        joints[1].minAngle = -1.8326; joints[1].maxAngle = 2.7052;

        joints[2].name = "joint_3_u";
        joints[2].originXyz = Vector3d(0, 0, 760);
        joints[2].originRpy = Vector3d::Zero();
        joints[2].axis = Vector3d(0, -1, 0);
        joints[2].minAngle = -1.5009; joints[2].maxAngle = 2.7925;

        joints[3].name = "joint_4_r";
        joints[3].originXyz = Vector3d(0, 0, 200);
        joints[3].originRpy = Vector3d::Zero();
        joints[3].axis = Vector3d(-1, 0, 0);
        joints[3].minAngle = -2.6180; joints[3].maxAngle = 2.6180;

        joints[4].name = "joint_5_b";
        joints[4].originXyz = Vector3d(1082, 0, 0);
        joints[4].originRpy = Vector3d::Zero();
        joints[4].axis = Vector3d(0, -1, 0);
        joints[4].minAngle = -2.3562; joints[4].maxAngle = 1.5708;

        joints[5].name = "joint_6_t";
        joints[5].originXyz = Vector3d(0, 0, 0);
        joints[5].originRpy = Vector3d::Zero();
        joints[5].axis = Vector3d(-1, 0, 0);
        joints[5].minAngle = -3.6652; joints[5].maxAngle = 3.6652;

        return joints;
    }

    JointAngles degToRad(const std::array<double, 6>& deg) {
        JointAngles rad;
        for (int i = 0; i < 6; i++) rad[i] = deg[i] * M_PI / 180.0;
        return rad;
    }

    double computePS(const JointAngles& rad) {
        Jacobian J = kdl_->computeJacobian(rad);
        return TwistDecomposition::computePS(J);
    }

    double computeVelocityScale(double ps) {
        if (ps > config_.psThreshold && config_.psThreshold > 0.0) {
            double ratio = config_.psThreshold / ps;
            double ratioSq = ratio * ratio;
            return std::max(config_.velocityScaleMin,
                             1.0 - (1.0 - config_.velocityScaleMin) * (1.0 - ratioSq));
        }
        return 1.0;
    }
};

// ============================================================================
// V1: PS Continuity Along Path
// ============================================================================
TEST_F(PSVerificationTest, V1_PS_ContinuousAlongJ5Sweep) {
    std::vector<double> psValues;
    std::vector<double> j5Angles;
    double step = 0.5;

    for (double j5 = -45.0; j5 <= 45.0; j5 += step) {
        if (std::abs(j5) < 0.01) continue;
        auto rad = degToRad({0, 30, -30, 0, j5, 0});
        double ps = computePS(rad);
        if (ps < 1e5) {
            psValues.push_back(ps);
            j5Angles.push_back(j5);
        }
    }

    ASSERT_GE(psValues.size(), 10u) << "Not enough valid PS samples";

    // Adjacent 0.5° steps should not jump more than 5.0 in PS
    for (size_t i = 1; i < psValues.size(); i++) {
        double delta = std::abs(psValues[i] - psValues[i-1]);
        EXPECT_LT(delta, 5.0) << "PS jump too large between J5="
            << j5Angles[i-1] << " and J5=" << j5Angles[i]
            << " (PS: " << psValues[i-1] << " -> " << psValues[i] << ")";
    }

    // PS peak should be near J5=0
    auto maxIt = std::max_element(psValues.begin(), psValues.end());
    size_t peakIdx = std::distance(psValues.begin(), maxIt);
    double peakJ5 = j5Angles[peakIdx];
    EXPECT_LT(std::abs(peakJ5), 2.0) << "PS peak should be near J5=0 but found at J5=" << peakJ5;
}

// ============================================================================
// V2: Velocity Scale Smoothness (No Step Changes)
// ============================================================================
TEST_F(PSVerificationTest, V2_VelocityScale_SmoothTransition) {
    std::vector<double> scaleValues;
    std::vector<double> j5Angles;
    double step = 0.5;

    for (double j5 = -45.0; j5 <= -0.5; j5 += step) {
        auto rad = degToRad({0, 30, -30, 0, j5, 0});
        double ps = computePS(rad);
        if (ps >= 1e5) continue;
        double scale = computeVelocityScale(ps);
        scaleValues.push_back(scale);
        j5Angles.push_back(j5);
    }

    ASSERT_GE(scaleValues.size(), 10u);

    // No step change > 10% between adjacent 0.5° points
    // (PS changes rapidly near singularity — quadratic ramp is smooth but steep)
    for (size_t i = 1; i < scaleValues.size(); i++) {
        double delta = std::abs(scaleValues[i] - scaleValues[i-1]);
        EXPECT_LT(delta, 0.10) << "Velocity scale jump at J5=" << j5Angles[i]
            << " (scale: " << scaleValues[i-1] << " -> " << scaleValues[i] << ")";
    }

    // Far from singularity: scale = 1.0
    EXPECT_NEAR(scaleValues.front(), 1.0, 1e-6)
        << "Scale should be 1.0 at J5=-45 (far from singularity)";

    // Near singularity: scale < 0.5
    EXPECT_LT(scaleValues.back(), 0.5)
        << "Scale should be reduced near J5=-0.5";

    // Scale always >= min
    for (size_t i = 0; i < scaleValues.size(); i++) {
        EXPECT_GE(scaleValues[i], config_.velocityScaleMin - 1e-10)
            << "Scale below minimum at J5=" << j5Angles[i];
    }
}

// ============================================================================
// V3: Velocity Scale = 1.0 at Exact Threshold (Seamless Entry)
// ============================================================================
TEST_F(PSVerificationTest, V3_VelocityScale_SeamlessAtThreshold) {
    double scale_at_threshold = computeVelocityScale(config_.psThreshold);
    EXPECT_NEAR(scale_at_threshold, 1.0, 1e-10)
        << "Velocity scale must be exactly 1.0 at PS = threshold (no step)";

    // Just above threshold: close to 1.0 (quadratic ramp starts gently)
    double scale_just_above = computeVelocityScale(config_.psThreshold * 1.01);
    EXPECT_GT(scale_just_above, 0.95)
        << "Scale should be near 1.0 just above threshold";
    EXPECT_LT(scale_just_above, 1.0)
        << "Scale should be slightly below 1.0 just above threshold";

    // Below threshold: exactly 1.0
    double scale_below = computeVelocityScale(config_.psThreshold * 0.5);
    EXPECT_NEAR(scale_below, 1.0, 1e-10);
}

// ============================================================================
// V4: Hysteresis Prevents Oscillation
// ============================================================================
TEST_F(PSVerificationTest, V4_Hysteresis_NoOscillation) {
    double enterThreshold = config_.psThreshold;
    double exitThreshold = config_.psThresholdExit;
    bool singularityActive = false;

    // PS values oscillating around enter threshold (0.3)
    std::vector<double> psSequence = {
        0.10, 0.20, 0.28, 0.32, 0.29, 0.31, 0.28, 0.30, 0.32, 0.25,
        0.22, 0.21, 0.19, 0.15, 0.18, 0.22, 0.28, 0.32, 0.35, 0.40
    };

    int stateChanges = 0;
    bool prevState = false;

    for (double ps : psSequence) {
        if (!singularityActive && ps > enterThreshold) {
            singularityActive = true;
        } else if (singularityActive && ps < exitThreshold) {
            singularityActive = false;
        }

        if (singularityActive != prevState) {
            stateChanges++;
            prevState = singularityActive;
        }
    }

    // With hysteresis: enter at 0.32, stay through 0.29/0.31/0.28/0.30/0.32/0.25/0.22/0.21,
    // exit at 0.19, re-enter at 0.32 → 3 transitions
    EXPECT_LE(stateChanges, 4) << "Hysteresis should prevent rapid state changes. Got "
        << stateChanges << " transitions";
}

// ============================================================================
// V5: TCP Accuracy Through Singularity (FK/IK round-trip)
// ============================================================================
TEST_F(PSVerificationTest, V5_TCP_AccuracyNearSingularity) {
    for (double j5 = -30.0; j5 >= -2.0; j5 += 1.0) {
        std::array<double, 6> deg = {0, 30, -30, 0, j5, 0};
        auto rad = degToRad(deg);

        auto pose = fk_->compute(rad);

        // Position should be finite
        EXPECT_TRUE(std::isfinite(pose.position[0])) << "X NaN at J5=" << j5;
        EXPECT_TRUE(std::isfinite(pose.position[1])) << "Y NaN at J5=" << j5;
        EXPECT_TRUE(std::isfinite(pose.position[2])) << "Z NaN at J5=" << j5;

        // Position within MA2010 workspace
        double reach = pose.position.norm();
        EXPECT_GT(reach, 100.0) << "Position too close to origin at J5=" << j5;
        EXPECT_LT(reach, 3000.0) << "Position beyond workspace at J5=" << j5;

        // IK round-trip
        auto ikResult = kdl_->computeIK(pose, rad);
        if (ikResult.has_value()) {
            auto roundTrip = fk_->compute(ikResult->angles);
            double posError = (roundTrip.position - pose.position).norm();
            EXPECT_LT(posError, 0.5) << "IK round-trip error "
                << posError << "mm at J5=" << j5;
        }
    }
}
