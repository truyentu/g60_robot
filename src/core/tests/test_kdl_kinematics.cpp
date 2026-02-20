/**
 * @file test_kdl_kinematics.cpp
 * @brief Unit tests for KDL Kinematics (FK cross-validation + IK round-trip)
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_02)
 */

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include "kinematics/UrdfForwardKinematics.hpp"
#include "kinematics/KDLKinematics.hpp"
#include <vector>
#include <random>

using namespace robot_controller::kinematics;

// ============================================================================
// MA2010 Joint Data (shared with test_urdf_fk.cpp)
// ============================================================================

static std::vector<UrdfJointDef> createMA2010Joints() {
    std::vector<UrdfJointDef> joints(6);

    joints[0].name = "joint_1_s";
    joints[0].originXyz = Vector3d(0, 0, 505);
    joints[0].originRpy = Vector3d(0, 0, 0);
    joints[0].axis = Vector3d(0, 0, 1);
    joints[0].minAngle = -3.1416;
    joints[0].maxAngle = 3.1416;

    joints[1].name = "joint_2_l";
    joints[1].originXyz = Vector3d(150, 0, 0);
    joints[1].originRpy = Vector3d(0, 0, 0);
    joints[1].axis = Vector3d(0, 1, 0);
    joints[1].minAngle = -1.8326;
    joints[1].maxAngle = 2.7052;

    joints[2].name = "joint_3_u";
    joints[2].originXyz = Vector3d(0, 0, 760);
    joints[2].originRpy = Vector3d(0, 0, 0);
    joints[2].axis = Vector3d(0, -1, 0);
    joints[2].minAngle = -1.5009;
    joints[2].maxAngle = 2.7925;

    joints[3].name = "joint_4_r";
    joints[3].originXyz = Vector3d(0, 0, 200);
    joints[3].originRpy = Vector3d(0, 0, 0);
    joints[3].axis = Vector3d(-1, 0, 0);
    joints[3].minAngle = -2.6180;
    joints[3].maxAngle = 2.6180;

    joints[4].name = "joint_5_b";
    joints[4].originXyz = Vector3d(1082, 0, 0);
    joints[4].originRpy = Vector3d(0, 0, 0);
    joints[4].axis = Vector3d(0, -1, 0);
    joints[4].minAngle = -2.3562;
    joints[4].maxAngle = 1.5708;

    joints[5].name = "joint_6_t";
    joints[5].originXyz = Vector3d(0, 0, 0);
    joints[5].originRpy = Vector3d(0, 0, 0);
    joints[5].axis = Vector3d(-1, 0, 0);
    joints[5].minAngle = -3.6652;
    joints[5].maxAngle = 3.6652;

    return joints;
}

static const Vector3d MA2010_TOOL_OFFSET(100, 0, 0);

// Test configurations
struct TestConfig {
    std::string name;
    JointAngles q;
};

static std::vector<TestConfig> getTestConfigs() {
    return {
        {"home",      {0, 0, 0, 0, 0, 0}},
        {"j1_90",     {M_PI/2, 0, 0, 0, 0, 0}},
        {"j1_neg90",  {-M_PI/2, 0, 0, 0, 0, 0}},
        {"j2_45",     {0, M_PI/4, 0, 0, 0, 0}},
        {"j3_45",     {0, 0, M_PI/4, 0, 0, 0}},
        {"j4_90",     {0, 0, 0, M_PI/2, 0, 0}},
        {"j5_45",     {0, 0, 0, 0, M_PI/4, 0}},
        {"j6_90",     {0, 0, 0, 0, 0, M_PI/2}},
        {"j12_combo", {M_PI/6, M_PI/4, 0, 0, 0, 0}},
        {"j123_combo",{M_PI/6, M_PI/4, -M_PI/6, 0, 0, 0}},
        {"all_small", {0.1, -0.2, 0.3, -0.1, 0.2, -0.3}},
        {"random_1",  {0.5, -0.3, 1.2, -0.7, 0.4, -1.1}},
        {"random_2",  {-1.0, 0.8, -0.5, 2.0, -1.5, 0.3}},
    };
}

// ============================================================================
// FK Cross-Validation Tests
// ============================================================================

class KDLvsURDF : public ::testing::Test {
protected:
    void SetUp() override {
        auto joints = createMA2010Joints();
        urdfFk_ = std::make_unique<UrdfForwardKinematics>(joints, MA2010_TOOL_OFFSET);
        kdlKin_ = std::make_unique<KDLKinematics>();
        kdlKin_->buildFromUrdfJoints(joints, MA2010_TOOL_OFFSET);
    }

    std::unique_ptr<UrdfForwardKinematics> urdfFk_;
    std::unique_ptr<KDLKinematics> kdlKin_;
};

TEST_F(KDLvsURDF, Initialized) {
    EXPECT_TRUE(kdlKin_->isInitialized());
}

TEST_F(KDLvsURDF, HomePosition_Match) {
    JointAngles q = {0, 0, 0, 0, 0, 0};
    auto urdfPose = urdfFk_->compute(q);
    auto kdlPose = kdlKin_->computeFK(q);

    double posErr = (urdfPose.position - kdlPose.position).norm();
    EXPECT_LT(posErr, 0.1)
        << "Home pos URDF: (" << urdfPose.position.transpose() << ")"
        << " KDL: (" << kdlPose.position.transpose() << ")"
        << " diff: " << posErr << "mm";

    // Orientation check
    Matrix3d R_err = urdfPose.rotation.transpose() * kdlPose.rotation;
    double oriErr = AngleAxisd(R_err).angle();
    EXPECT_LT(oriErr, 0.01)
        << "Home orientation diff: " << oriErr << " rad";
}

TEST_F(KDLvsURDF, AllConfigs_Match) {
    auto configs = getTestConfigs();

    for (const auto& cfg : configs) {
        auto urdfPose = urdfFk_->compute(cfg.q);
        auto kdlPose = kdlKin_->computeFK(cfg.q);

        double posErr = (urdfPose.position - kdlPose.position).norm();
        EXPECT_LT(posErr, 0.1)
            << "Config: " << cfg.name
            << " pos URDF: (" << urdfPose.position.transpose() << ")"
            << " KDL: (" << kdlPose.position.transpose() << ")"
            << " diff: " << posErr << "mm";

        Matrix3d R_err = urdfPose.rotation.transpose() * kdlPose.rotation;
        double oriErr = AngleAxisd(R_err).angle();
        EXPECT_LT(oriErr, 0.01)
            << "Config: " << cfg.name
            << " ori diff: " << oriErr << " rad";
    }
}

// ============================================================================
// IK Round-Trip Tests
// ============================================================================

class KDLIK : public ::testing::Test {
protected:
    void SetUp() override {
        auto joints = createMA2010Joints();
        urdfFk_ = std::make_unique<UrdfForwardKinematics>(joints, MA2010_TOOL_OFFSET);
        kdlKin_ = std::make_unique<KDLKinematics>();
        kdlKin_->buildFromUrdfJoints(joints, MA2010_TOOL_OFFSET);

        // Set joint limits
        std::vector<std::pair<double,double>> limits;
        for (const auto& j : joints) {
            limits.emplace_back(j.minAngle, j.maxAngle);
        }
        kdlKin_->setJointLimits(limits);
    }

    std::unique_ptr<UrdfForwardKinematics> urdfFk_;
    std::unique_ptr<KDLKinematics> kdlKin_;
};

TEST_F(KDLIK, RoundTrip_HomePosition) {
    JointAngles q_original = {0, 0, 0, 0, 0, 0};
    auto targetPose = urdfFk_->compute(q_original);

    auto solution = kdlKin_->computeIK(targetPose, q_original);
    ASSERT_TRUE(solution.has_value()) << "IK failed for home position";

    auto recoveredPose = urdfFk_->compute(solution->angles);
    double posErr = (targetPose.position - recoveredPose.position).norm();
    EXPECT_LT(posErr, 0.1) << "Round-trip pos err: " << posErr << "mm";

    Matrix3d R_err = targetPose.rotation.transpose() * recoveredPose.rotation;
    double oriErr = AngleAxisd(R_err).angle();
    EXPECT_LT(oriErr, 0.01) << "Round-trip ori err: " << oriErr << " rad";
}

TEST_F(KDLIK, RoundTrip_AllConfigs) {
    auto configs = getTestConfigs();

    for (const auto& cfg : configs) {
        auto targetPose = urdfFk_->compute(cfg.q);

        // IK with original angles as seed (best case)
        auto solution = kdlKin_->computeIK(targetPose, cfg.q);
        ASSERT_TRUE(solution.has_value())
            << "IK failed for config: " << cfg.name;

        auto recoveredPose = urdfFk_->compute(solution->angles);
        double posErr = (targetPose.position - recoveredPose.position).norm();
        EXPECT_LT(posErr, 0.5)
            << "Config: " << cfg.name << " round-trip pos err: " << posErr << "mm";
    }
}

TEST_F(KDLIK, RoundTrip_DifferentSeed) {
    // IK from a nearby seed (realistic jog scenario — seed is close but not exact)
    JointAngles q_original = {0.5, -0.3, 1.2, -0.7, 0.4, -1.1};
    auto targetPose = urdfFk_->compute(q_original);

    // Use a seed close to original (simulates jog: seed = previous position)
    JointAngles q_seed = {0.45, -0.25, 1.15, -0.65, 0.35, -1.05};
    auto solution = kdlKin_->computeIK(targetPose, q_seed);
    ASSERT_TRUE(solution.has_value())
        << "IK failed with nearby seed";

    auto recoveredPose = urdfFk_->compute(solution->angles);
    double posErr = (targetPose.position - recoveredPose.position).norm();
    EXPECT_LT(posErr, 1.0)
        << "Different seed round-trip pos err: " << posErr << "mm";
}

TEST_F(KDLIK, SolveRate_500Random) {
    std::mt19937 rng(42);  // Deterministic seed
    auto joints = createMA2010Joints();

    // Generate distributions within joint limits
    std::vector<std::uniform_real_distribution<double>> dists;
    for (const auto& j : joints) {
        // Use slightly tighter limits for safety
        double margin = 0.1;
        dists.emplace_back(j.minAngle + margin, j.maxAngle - margin);
    }

    int success = 0;
    int total = 500;

    for (int i = 0; i < total; i++) {
        JointAngles q_random;
        for (int j = 0; j < 6; j++) {
            q_random[j] = dists[j](rng);
        }

        auto pose = urdfFk_->compute(q_random);
        auto sol = kdlKin_->computeIK(pose, q_random);
        if (sol.has_value()) {
            // Verify recovered pose matches
            auto recovered = urdfFk_->compute(sol->angles);
            double posErr = (pose.position - recovered.position).norm();
            if (posErr < 1.0) {  // < 1mm acceptable
                success++;
            }
        }
    }

    double rate = 100.0 * success / total;
    EXPECT_GT(rate, 85.0)
        << "IK solve rate: " << rate << "% (" << success << "/" << total << ")";
}

TEST_F(KDLIK, CartesianJogSimulation) {
    // Simulate Cartesian jog: small steps in World X
    JointAngles q = {0, 0, 0, 0, 0, 0};  // home
    double stepSize = 5.0;  // 5mm per step
    int numSteps = 20;
    int success = 0;

    for (int i = 0; i < numSteps; i++) {
        auto currentPose = urdfFk_->compute(q);

        // Jog World X+
        TCPPose targetPose = currentPose;
        targetPose.position.x() += stepSize;

        auto solution = kdlKin_->computeIK(targetPose, q);
        if (solution.has_value()) {
            auto recovered = urdfFk_->compute(solution->angles);
            double posErr = (targetPose.position - recovered.position).norm();
            if (posErr < 0.5) {
                q = solution->angles;  // Use solution as next seed
                success++;
            } else {
                break;
            }
        } else {
            break;
        }
    }

    EXPECT_GE(success, 10)
        << "Cartesian jog simulation: " << success << "/" << numSteps << " steps succeeded";
}

// ============================================================================
// Jacobian Test
// ============================================================================

TEST_F(KDLvsURDF, Jacobian_MatchesNumerical) {
    JointAngles q = {0.1, -0.2, 0.3, -0.1, 0.2, -0.3};

    auto kdlJac = kdlKin_->computeJacobian(q);
    auto urdfJac = urdfFk_->computeJacobian(q);

    // Compare element-wise
    double maxDiff = 0;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double diff = std::abs(kdlJac(i, j) - urdfJac(i, j));
            maxDiff = std::max(maxDiff, diff);
        }
    }

    // Allow larger tolerance since different FK implementations
    EXPECT_LT(maxDiff, 1.0)
        << "Jacobian max element diff: " << maxDiff;
}
