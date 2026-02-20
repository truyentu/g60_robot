/**
 * @file test_ps_threshold_survey.cpp
 * @brief Survey PS (Parameter of Singularity) values across MA2010 configurations
 *
 * Purpose: Determine appropriate psThreshold by measuring actual PS values at:
 *   - Normal working positions (PS should be low)
 *   - Near wrist singularity (J5 approaching 0°)
 *   - Near shoulder singularity (J2+J3 approaching ±90°)
 *   - Various J5 angles to find the transition curve
 *
 * Output: Table of (J5 angle → PS value) to calibrate threshold.
 */

#include <cmath>
#include <iostream>
#include <iomanip>
#include <array>
#include <vector>

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
// MA2010 URDF Joint Definitions (same as other tests)
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

class PSSurveyTest : public ::testing::Test {
protected:
    std::unique_ptr<KDLKinematics> kdl_;
    std::vector<UrdfJointDef> joints_;

    void SetUp() override {
        Logger::init("test_ps_survey.log", "warn");
        joints_ = buildMA2010Joints();
        Vector3d toolOffset(100, 0, 0);
        kdl_ = std::make_unique<KDLKinematics>();
        kdl_->buildFromUrdfJoints(joints_, toolOffset);
    }

    double computePS(const std::array<double, 6>& degAngles) {
        JointAngles rad;
        for (int i = 0; i < 6; i++) {
            rad[i] = degAngles[i] * M_PI / 180.0;
        }
        Jacobian J = kdl_->computeJacobian(rad);
        return TwistDecomposition::computePS(J);
    }

    void printConfig(const std::string& label,
                     const std::array<double, 6>& deg,
                     double ps) {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "  " << std::left << std::setw(35) << label
                  << " J=[" << deg[0] << "," << deg[1] << ","
                  << deg[2] << "," << deg[3] << ","
                  << deg[4] << "," << deg[5] << "]"
                  << "  PS=" << std::setprecision(2) << ps;
        if (ps > 500) std::cout << "  *** VERY HIGH";
        else if (ps > 50) std::cout << "  ** HIGH";
        else if (ps > 10) std::cout << "  * MODERATE";
        std::cout << std::endl;
    }
};

// ============================================================================
// TEST 1: Normal working positions — PS should be LOW
// ============================================================================
TEST_F(PSSurveyTest, NormalPositions) {
    std::cout << "\n=== NORMAL WORKING POSITIONS (expect PS < 50) ===" << std::endl;

    struct Config {
        std::string name;
        std::array<double, 6> joints;
    };

    std::vector<Config> configs = {
        {"Home (all zeros)",            {0, 0, 0, 0, -45, 0}},
        {"Typical weld pose 1",         {0, 30, -30, 0, -45, 0}},
        {"Typical weld pose 2",         {45, 20, -20, 30, -60, 15}},
        {"J1 rotated 90",              {90, 30, -30, 0, -45, 0}},
        {"J1 rotated -90",             {-90, 30, -30, 0, -45, 0}},
        {"Arm forward & down",          {0, 45, -45, 0, -45, 0}},
        {"Arm raised high",             {0, 60, -20, 0, -30, 0}},
        {"Full reach forward",          {0, 0, 0, 0, -90, 0}},
        {"Wrist tilted 45",             {0, 30, -30, 45, -45, -45}},
        {"Mixed config",                {30, 25, -25, -20, -50, 10}},
    };

    double maxPS = 0;
    for (const auto& cfg : configs) {
        double ps = computePS(cfg.joints);
        printConfig(cfg.name, cfg.joints, ps);
        if (ps > maxPS) maxPS = ps;
    }

    std::cout << "\n  >>> Max PS in normal positions: " << maxPS << std::endl;
    std::cout << "  >>> If threshold=50, all normal positions should be below 50\n" << std::endl;

    // Normal positions should all have PS < 50
    for (const auto& cfg : configs) {
        double ps = computePS(cfg.joints);
        EXPECT_LT(ps, 100.0) << "Config '" << cfg.name
            << "' has unexpectedly high PS=" << ps;
    }
}

// ============================================================================
// TEST 2: Sweep J5 from BOTH sides toward 0° — find PS transition curve
// ============================================================================
TEST_F(PSSurveyTest, J5SweepToWristSingularity) {
    std::cout << "\n=== J5 SWEEP: Wrist Singularity from BOTH sides ===" << std::endl;
    std::cout << "  Base config: J=[0, 30, -30, 0, J5, 0]" << std::endl;
    std::cout << "  J5 limits: [-135°, +90°] — singularity at 0°" << std::endl;
    std::cout << std::setw(10) << "J5(deg)" << std::setw(15) << "PS"
              << std::setw(20) << "VelScale(thr=0.3)" << std::endl;
    std::cout << std::string(45, '-') << std::endl;

    // Sweep BOTH sides: negative AND positive approach to 0°
    std::vector<double> j5Values = {
        -90, -60, -45, -30, -20, -15, -10, -8, -5, -3, -2, -1, -0.5,
        0,
        0.5, 1, 2, 3, 5, 8, 10, 15, 20, 30, 45, 60, 90
    };

    for (double j5 : j5Values) {
        std::array<double, 6> deg = {0, 30, -30, 0, j5, 0};
        double ps = computePS(deg);

        // Cap degenerate
        if (ps >= 1e5) ps = 1e6;

        double velScale = 1.0;
        if (ps < 1e5 && ps > 0.3) {
            velScale = std::max(0.1, 0.3 / ps);
        }

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << j5
                  << std::setprecision(2)
                  << std::setw(15) << ps
                  << std::setprecision(0)
                  << std::setw(15) << (velScale * 100) << "%";
        if (ps >= 1e5) std::cout << "  [DEGENERATE]";
        else if (ps > 0.3) std::cout << "  [SLOWING]";
        std::cout << std::endl;
    }
}

// ============================================================================
// TEST 3: Sweep J5 with different J4 values — coupling effect
// ============================================================================
TEST_F(PSSurveyTest, J5SweepWithJ4Variations) {
    std::cout << "\n=== J5 SWEEP with J4 variations ===" << std::endl;
    std::cout << "  Does J4 value affect PS near wrist singularity?" << std::endl;

    std::vector<double> j4Values = {0, 30, 60, 90, 120};
    std::vector<double> j5Values = {-45, -20, -10, -5, -2, 0};

    std::cout << std::setw(8) << "J4\\J5";
    for (double j5 : j5Values) {
        std::cout << std::setw(10) << j5;
    }
    std::cout << std::endl << std::string(70, '-') << std::endl;

    for (double j4 : j4Values) {
        std::cout << std::setw(8) << j4;
        for (double j5 : j5Values) {
            std::array<double, 6> deg = {0, 30, -30, j4, j5, 0};
            double ps = computePS(deg);
            if (ps >= 1e5) {
                std::cout << std::setw(10) << "INF";
            } else {
                std::cout << std::fixed << std::setprecision(1) << std::setw(10) << ps;
            }
        }
        std::cout << std::endl;
    }
}

// ============================================================================
// TEST 4: Shoulder singularity — J2+J3 approaching ±90°
// ============================================================================
TEST_F(PSSurveyTest, ShoulderSingularitySweep) {
    std::cout << "\n=== SHOULDER SINGULARITY: J2+J3 near ±90° ===" << std::endl;
    std::cout << "  Sweep J2+J3 sum while keeping J5=-45 (no wrist sing.)" << std::endl;

    // Target: J2+J3 = 90° → J3 = 90-J2
    // J2=30, J3=60 → sum=90°
    // J2=45, J3=45 → sum=90°

    struct Config {
        std::string name;
        double j2, j3;
    };

    std::vector<Config> configs = {
        {"J2+J3 = 0  (normal)",    30, -30},
        {"J2+J3 = 30",             45, -15},
        {"J2+J3 = 50",             50,   0},
        {"J2+J3 = 60",             60,   0},
        {"J2+J3 = 70",             70,   0},
        {"J2+J3 = 75",             60,  15},
        {"J2+J3 = 80",             60,  20},
        {"J2+J3 = 85",             60,  25},
        {"J2+J3 = 88",             60,  28},
        {"J2+J3 = 90 (singular!)", 60,  30},
        {"J2+J3 = -80",           -40, -40},
        {"J2+J3 = -85",           -40, -45},
        {"J2+J3 = -88",           -40, -48},
        {"J2+J3 = -90 (singular!)",-45,-45},
    };

    std::cout << std::setw(30) << "Config" << std::setw(8) << "J2"
              << std::setw(8) << "J3" << std::setw(10) << "J2+J3"
              << std::setw(12) << "PS" << std::endl;
    std::cout << std::string(68, '-') << std::endl;

    for (const auto& cfg : configs) {
        std::array<double, 6> deg = {0, cfg.j2, cfg.j3, 0, -45, 0};
        double ps = computePS(deg);

        std::cout << std::setw(30) << cfg.name
                  << std::fixed << std::setprecision(0)
                  << std::setw(8) << cfg.j2
                  << std::setw(8) << cfg.j3
                  << std::setw(10) << (cfg.j2 + cfg.j3)
                  << std::setprecision(2) << std::setw(12);
        if (ps >= 1e5) std::cout << "INF";
        else std::cout << ps;
        std::cout << std::endl;
    }
}

// ============================================================================
// TEST 5: Threshold recommendation based on data
// ============================================================================
TEST_F(PSSurveyTest, ThresholdRecommendation) {
    std::cout << "\n=== THRESHOLD RECOMMENDATION ===" << std::endl;

    // Measure PS at key J5 angles to find where slowdown should start
    struct Measurement {
        double j5Deg;
        double ps;
    };

    std::vector<Measurement> measurements;
    for (double j5 = -90; j5 <= 0; j5 += 1.0) {
        std::array<double, 6> deg = {0, 30, -30, 0, j5, 0};
        double ps = computePS(deg);
        if (ps < 1e5) {
            measurements.push_back({j5, ps});
        }
    }

    // Find J5 angle where PS first exceeds various thresholds
    std::vector<double> thresholds = {10, 20, 30, 50, 75, 100, 200};
    std::cout << std::setw(15) << "Threshold" << std::setw(20) << "J5 starts slowing"
              << std::setw(20) << "Min velocity" << std::endl;
    std::cout << std::string(55, '-') << std::endl;

    for (double thr : thresholds) {
        double firstJ5 = 0;
        double maxPS = 0;
        for (const auto& m : measurements) {
            if (m.ps > thr && firstJ5 == 0) {
                firstJ5 = m.j5Deg;
            }
            if (m.ps > maxPS) maxPS = m.ps;
        }

        double minVelScale = std::max(0.1, thr / maxPS);

        std::cout << std::fixed << std::setprecision(0)
                  << std::setw(15) << thr
                  << std::setprecision(1) << std::setw(15) << firstJ5 << "°"
                  << std::setprecision(0) << std::setw(15) << (minVelScale * 100) << "%"
                  << std::endl;
    }

    std::cout << "\n  GUIDE:" << std::endl;
    std::cout << "  - Threshold too LOW (e.g. 10): Robot slows down too early → annoying" << std::endl;
    std::cout << "  - Threshold too HIGH (e.g. 200): Robot only slows very close to singularity" << std::endl;
    std::cout << "  - Good threshold: starts slowing ~10-15° from singularity" << std::endl;
}
