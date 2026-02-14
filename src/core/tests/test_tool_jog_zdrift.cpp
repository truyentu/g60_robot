/**
 * Diagnostic test: Tool Frame Jog Z-Drift Investigation
 *
 * Simulates jogging in Tool Frame (frame=2) along X-axis
 * and checks if Z-drift occurs in world coordinates.
 *
 * Uses real MA2010 URDF data + active tool TCP config.
 */

#include <iostream>
#include <iomanip>
#include <cmath>

#define _USE_MATH_DEFINES
#include <math.h>

#include "../src/kinematics/UrdfForwardKinematics.hpp"
#include "../src/kinematics/MathTypes.hpp"
#include "../src/tool/ToolTypes.hpp"

using namespace robot_controller::kinematics;
using namespace robot_controller::tool;

// MA2010 URDF joint definitions (from robot.yaml)
std::vector<UrdfJointDef> buildMA2010Joints() {
    std::vector<UrdfJointDef> joints(6);

    joints[0].name = "A1"; joints[0].originXyz = Vector3d(0, 0, 505);
    joints[0].originRpy = Vector3d::Zero(); joints[0].axis = Vector3d(0, 0, 1);

    joints[1].name = "A2"; joints[1].originXyz = Vector3d(150, 0, 0);
    joints[1].originRpy = Vector3d::Zero(); joints[1].axis = Vector3d(0, 1, 0);

    joints[2].name = "A3"; joints[2].originXyz = Vector3d(0, 0, 760);
    joints[2].originRpy = Vector3d::Zero(); joints[2].axis = Vector3d(0, -1, 0);

    joints[3].name = "A4"; joints[3].originXyz = Vector3d(0, 0, 200);
    joints[3].originRpy = Vector3d::Zero(); joints[3].axis = Vector3d(-1, 0, 0);

    joints[4].name = "A5"; joints[4].originXyz = Vector3d(1082, 0, 0);
    joints[4].originRpy = Vector3d::Zero(); joints[4].axis = Vector3d(0, -1, 0);

    joints[5].name = "A6"; joints[5].originXyz = Vector3d(0, 0, 0);
    joints[5].originRpy = Vector3d::Zero(); joints[5].axis = Vector3d(-1, 0, 0);

    return joints;
}

void printMatrix3d(const std::string& label, const Matrix3d& R) {
    std::cout << label << ":\n";
    for (int i = 0; i < 3; i++) {
        std::cout << "  [" << std::fixed << std::setprecision(6)
                  << std::setw(10) << R(i,0) << ", "
                  << std::setw(10) << R(i,1) << ", "
                  << std::setw(10) << R(i,2) << "]\n";
    }
}

int main() {
    std::cout << "=== Tool Frame Jog Z-Drift Diagnostic ===\n\n";

    // Build MA2010 kinematics
    Vector3d flangeOffset(100, 0, 0);  // from robot.yaml flange.offset
    auto joints = buildMA2010Joints();
    UrdfForwardKinematics urdfFk(joints, flangeOffset);

    // Active tool TCP (from tool_20260210174142.yaml)
    ToolTCP toolTcp;
    toolTcp.x = 243.42; toolTcp.y = -0.03; toolTcp.z = -96.02;
    toolTcp.rx = 0; toolTcp.ry = 0; toolTcp.rz = 0;
    Eigen::Matrix4d T_tool = toolTcp.toTransform();

    std::cout << "--- Configuration ---\n";
    std::cout << "URDF FK flange_offset: (" << flangeOffset.x() << ", "
              << flangeOffset.y() << ", " << flangeOffset.z() << ")\n";
    std::cout << "ToolTCP (active): x=" << toolTcp.x << " y=" << toolTcp.y
              << " z=" << toolTcp.z << " rx=" << toolTcp.rx
              << " ry=" << toolTcp.ry << " rz=" << toolTcp.rz << "\n\n";

    // Test with several representative joint configurations
    std::vector<std::pair<std::string, JointAngles>> configs = {
        {"Home (all zeros)", {0, 0, 0, 0, 0, 0}},
        {"J2=-45°", {0, -45*M_PI/180, 0, 0, 0, 0}},
        {"J2=-90° (arm forward)", {0, -90*M_PI/180, 0, 0, 0, 0}},
        {"J5=-90° (wrist bent)", {0, 0, 0, 0, -90*M_PI/180, 0}},
        {"J2=-45 J5=-45 (typical pose)", {0, -45*M_PI/180, 0, 0, -45*M_PI/180, 0}},
        {"J1=30 J2=-60 J5=-30", {30*M_PI/180, -60*M_PI/180, 0, 0, -30*M_PI/180, 0}},
    };

    for (auto& [name, jointRad] : configs) {
        std::cout << "=== Config: " << name << " ===\n";

        // Step 1: URDF FK (returns "flange" with toolOffset)
        auto flangePose = urdfFk.compute(jointRad);

        // Step 2: True TCP = T_flange * T_tool
        Eigen::Matrix4d T_flange = flangePose.toTransform();
        Eigen::Matrix4d T_tcp = T_flange * T_tool;
        TCPPose tcpPose = TCPPose::fromTransform(T_tcp);

        std::cout << "  FK 'flange' pos: (" << std::fixed << std::setprecision(2)
                  << flangePose.position.x() << ", " << flangePose.position.y()
                  << ", " << flangePose.position.z() << ")\n";
        std::cout << "  FK 'flange' RPY deg: ("
                  << flangePose.rpy[0]*180/M_PI << ", " << flangePose.rpy[1]*180/M_PI
                  << ", " << flangePose.rpy[2]*180/M_PI << ")\n";
        std::cout << "  TCP pos: ("
                  << tcpPose.position.x() << ", " << tcpPose.position.y()
                  << ", " << tcpPose.position.z() << ")\n";
        std::cout << "  TCP RPY deg: ("
                  << tcpPose.rpy[0]*180/M_PI << ", " << tcpPose.rpy[1]*180/M_PI
                  << ", " << tcpPose.rpy[2]*180/M_PI << ")\n";

        // Step 3: Check R_tcp columns (tool axes in world frame)
        Matrix3d R_tcp = tcpPose.rotation;
        printMatrix3d("  R_tcp (TCP rotation in world)", R_tcp);

        // Step 4: Simulate Tool X+ jog
        double delta = 10.0; // 10mm
        Vector3d delta_tool(delta, 0, 0);
        Vector3d delta_world = R_tcp * delta_tool;

        std::cout << "\n  Tool X+ jog (10mm):\n";
        std::cout << "    R_tcp col0 (Tool X in world): ("
                  << R_tcp(0,0) << ", " << R_tcp(1,0) << ", " << R_tcp(2,0) << ")\n";
        std::cout << "    delta_world: (" << delta_world.x() << ", " << delta_world.y()
                  << ", " << delta_world.z() << ")\n";
        std::cout << "    ** Z-drift per 10mm X-jog: " << delta_world.z() << " mm **\n";

        if (std::abs(delta_world.z()) > 0.01) {
            std::cout << "    >>> WARNING: Z-DRIFT DETECTED! Tool X is NOT parallel to ground!\n";
            double tiltDeg = std::asin(std::abs(R_tcp(2,0))) * 180.0 / M_PI;
            std::cout << "    >>> Tool X tilt from horizontal: " << tiltDeg << " degrees\n";
        } else {
            std::cout << "    OK: Tool X is parallel to ground (Z-drift < 0.01mm)\n";
        }

        // Step 5: Check potential double-counting
        // URDF FK toolOffset contributes: flange_offset = (100,0,0) in tool frame
        // Then T_tool contributes: tcp = (243.42, -0.03, -96.02) in flange frame
        // Are these overlapping?

        std::cout << "\n";
    }

    // === KEY DIAGNOSTIC: Check URDF FK at zeros vs pure chain ===
    std::cout << "=== Double-Counting Check ===\n";
    UrdfForwardKinematics urdfFk_noOffset(joints, Vector3d::Zero()); // NO tool offset
    JointAngles zeros{};
    auto fkWithOffset = urdfFk.compute(zeros);
    auto fkNoOffset = urdfFk_noOffset.compute(zeros);

    std::cout << "FK with flange_offset(100,0,0): pos=("
              << fkWithOffset.position.x() << ", " << fkWithOffset.position.y()
              << ", " << fkWithOffset.position.z() << ")\n";
    std::cout << "FK without flange_offset:       pos=("
              << fkNoOffset.position.x() << ", " << fkNoOffset.position.y()
              << ", " << fkNoOffset.position.z() << ")\n";
    std::cout << "Difference (should be flange_offset in world): ("
              << fkWithOffset.position.x() - fkNoOffset.position.x() << ", "
              << fkWithOffset.position.y() - fkNoOffset.position.y() << ", "
              << fkWithOffset.position.z() - fkNoOffset.position.z() << ")\n";

    // Now apply T_tool on top of FK-with-offset
    Eigen::Matrix4d T_fk = fkWithOffset.toTransform();
    Eigen::Matrix4d T_tcp_final = T_fk * T_tool;
    TCPPose tcpFinal = TCPPose::fromTransform(T_tcp_final);

    // And apply T_tool on FK-without-offset (what it SHOULD be)
    Eigen::Matrix4d T_fk_noOff = fkNoOffset.toTransform();
    Eigen::Matrix4d T_tcp_correct = T_fk_noOff * T_tool;
    TCPPose tcpCorrect = TCPPose::fromTransform(T_tcp_correct);

    std::cout << "\nTCP = FK_withOffset * T_tool:    pos=("
              << tcpFinal.position.x() << ", " << tcpFinal.position.y()
              << ", " << tcpFinal.position.z() << ")\n";
    std::cout << "TCP = FK_noOffset * T_tool:      pos=("
              << tcpCorrect.position.x() << ", " << tcpCorrect.position.y()
              << ", " << tcpCorrect.position.z() << ")\n";
    std::cout << "Difference (double-count error):  ("
              << tcpFinal.position.x() - tcpCorrect.position.x() << ", "
              << tcpFinal.position.y() - tcpCorrect.position.y() << ", "
              << tcpFinal.position.z() - tcpCorrect.position.z() << ")\n";

    std::cout << "\nNote: If 'Difference' is ~(100,0,0) in world frame,\n"
              << "then flange_offset is being double-counted!\n"
              << "(Once in URDF FK, once in T_tool translation)\n";

    return 0;
}
