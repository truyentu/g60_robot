# CORE MODULE: KINEMATICS
## Forward & Inverse Kinematics Engine

**Version:** 1.0
**Ngày tạo:** 01/02/2026
**Trạng thái:** DRAFT
**Module Type:** Core Library (C++)
**Dependencies:** Eigen3, Robotics Library (optional)

---

## 1. TỔNG QUAN MODULE

### 1.1 Mục đích
Module Kinematics là trái tim toán học của Robot Controller, chịu trách nhiệm:
- **Forward Kinematics (FK):** Tính vị trí TCP từ góc các khớp
- **Inverse Kinematics (IK):** Tính góc các khớp từ vị trí TCP mong muốn
- **Jacobian:** Ánh xạ vận tốc giữa không gian khớp và không gian Cartesian
- **Configuration Management:** Quản lý 8 cấu hình nghiệm IK

### 1.2 Yêu cầu Kỹ thuật

| Yêu cầu | Giá trị | Ghi chú |
|---------|---------|---------|
| Độ chính xác FK | < 0.01mm | Position error |
| Độ chính xác IK | < 0.01mm | Với valid solution |
| Thời gian tính FK | < 10μs | Per calculation |
| Thời gian tính IK | < 50μs | Analytical solution |
| Hỗ trợ cấu hình | 8 solutions | ARM/ELBOW/WRIST flags |

### 1.3 Kiến trúc Module

```
┌─────────────────────────────────────────────────────────────────┐
│                    KINEMATICS MODULE                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    RobotModel                             │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐   │   │
│  │  │ DH Params   │  │Joint Limits │  │  TCP Offset     │   │   │
│  │  │ (6 joints)  │  │ (min/max)   │  │  (Tool Frame)   │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│              ┌───────────────┼───────────────┐                  │
│              │               │               │                  │
│              ▼               ▼               ▼                  │
│  ┌─────────────────┐ ┌─────────────┐ ┌─────────────────────┐   │
│  │    Forward      │ │  Inverse    │ │     Jacobian        │   │
│  │   Kinematics    │ │ Kinematics  │ │    Calculator       │   │
│  │                 │ │             │ │                     │   │
│  │  q[6] → T[4x4]  │ │ T[4x4] → q[]│ │  q[6] → J[6x6]     │   │
│  └────────┬────────┘ └──────┬──────┘ └──────────┬──────────┘   │
│           │                 │                    │              │
│           └─────────────────┼────────────────────┘              │
│                             │                                   │
│                             ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  KinematicsEngine                         │   │
│  │                                                           │   │
│  │  • Unified interface for all kinematics operations       │   │
│  │  • Configuration selection (ARM/ELBOW/WRIST)             │   │
│  │  • Joint limit validation                                │   │
│  │  • Singularity detection                                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. LÝ THUYẾT CƠ BẢN

### 2.1 Quy ước Denavit-Hartenberg (DH)

Module sử dụng quy ước DH Chuẩn (Standard DH Convention):

```
Ma trận biến đổi từ frame i-1 đến frame i:

T(i-1,i) = Rz(θi) × Tz(di) × Tx(ai) × Rx(αi)

        ┌                                              ┐
        │ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) │
T =     │ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) │
        │   0         sin(α)         cos(α)          d     │
        │   0           0              0             1     │
        └                                              ┘
```

**Tham số DH:**
| Tham số | Ký hiệu | Ý nghĩa | Đơn vị |
|---------|---------|---------|--------|
| Joint angle | θ (theta) | Góc quay khớp | rad |
| Link offset | d | Khoảng cách dọc trục Z | mm |
| Link length | a | Khoảng cách dọc trục X | mm |
| Link twist | α (alpha) | Góc xoắn giữa 2 trục Z | rad |

### 2.2 Cấu trúc Robot PUMA-like (6-DOF)

```
        Z0 (Base)
        │
        │ Joint 1 (Revolute)
        │
    ┌───┴───┐
    │ Link1 │ d1 (height)
    └───┬───┘
        │
   ─────┼───── Z1
        │
   Joint 2 ───────────────┐
        │                 │
        │                 │ a2 (upper arm)
        │                 │
   Joint 3 ───────────────┘
        │
        │ a3 (forearm)
        │
   Joint 4 ─── Wrist ─────┬─── Joint 5 ─── Joint 6 ─── TCP
        │                 │
        └─────────────────┘
         d4 (wrist offset)
```

**Điều kiện Pieper (Analytical IK có thể giải được):**
1. Trục 4, 5, 6 giao nhau tại một điểm (Spherical Wrist)
2. Trục 2 // Trục 3
3. Trục 1 ⊥ Trục 2

### 2.3 Bảng DH Parameters mẫu (PUMA 560)

| Joint | θ (offset) | d (mm) | a (mm) | α (rad) |
|-------|------------|--------|--------|---------|
| 1 | 0 | 0 | 0 | -π/2 |
| 2 | 0 | 149.09 | 431.80 | 0 |
| 3 | 0 | 0 | -20.32 | π/2 |
| 4 | 0 | 433.07 | 0 | -π/2 |
| 5 | 0 | 0 | 0 | π/2 |
| 6 | 0 | 56.25 | 0 | 0 |

---

## 3. CẤU TRÚC DỮ LIỆU

### 3.1 DH Parameter Structure

```cpp
// DHParameters.hpp
#pragma once

#include <array>
#include <cmath>

namespace robot_controller::kinematics {

struct DHParameter {
    double theta_offset;  // Joint angle offset (rad) - added to joint variable
    double d;             // Link offset (mm)
    double a;             // Link length (mm)
    double alpha;         // Link twist (rad)

    // Default constructor
    DHParameter() : theta_offset(0), d(0), a(0), alpha(0) {}

    // Parameterized constructor
    DHParameter(double theta_off, double d_val, double a_val, double alpha_val)
        : theta_offset(theta_off), d(d_val), a(a_val), alpha(alpha_val) {}
};

// 6-DOF robot DH parameters
using DHParameterSet = std::array<DHParameter, 6>;

// Joint limits
struct JointLimit {
    double min;  // Minimum angle (rad)
    double max;  // Maximum angle (rad)

    bool isWithinLimit(double value) const {
        return value >= min && value <= max;
    }

    double clamp(double value) const {
        return std::max(min, std::min(max, value));
    }
};

using JointLimitSet = std::array<JointLimit, 6>;

// Joint angles (6-DOF)
using JointAngles = std::array<double, 6>;

} // namespace robot_controller::kinematics
```

### 3.2 Pose & Transform Structures

```cpp
// Pose.hpp
#pragma once

#include <Eigen/Dense>
#include <array>

namespace robot_controller::kinematics {

// 4x4 Homogeneous transformation matrix
using Transform = Eigen::Matrix4d;

// 3x3 Rotation matrix
using RotationMatrix = Eigen::Matrix3d;

// 3D Vector
using Vector3 = Eigen::Vector3d;

// 6D Vector (position + orientation or velocity)
using Vector6 = Eigen::Matrix<double, 6, 1>;

// Cartesian pose (position + Euler angles)
struct CartesianPose {
    double x, y, z;        // Position (mm)
    double rx, ry, rz;     // Orientation (rad) - ZYX Euler angles

    CartesianPose() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}

    CartesianPose(double px, double py, double pz,
                  double roll, double pitch, double yaw)
        : x(px), y(py), z(pz), rx(roll), ry(pitch), rz(yaw) {}

    // Conversion to/from Transform
    Transform toTransform() const;
    static CartesianPose fromTransform(const Transform& T);

    // Position vector
    Vector3 position() const { return Vector3(x, y, z); }

    // Orientation as Euler angles
    Vector3 orientation() const { return Vector3(rx, ry, rz); }
};

// Quaternion representation (for interpolation)
struct QuaternionPose {
    double x, y, z;     // Position (mm)
    double qw, qx, qy, qz;  // Quaternion (w, x, y, z)

    static QuaternionPose fromTransform(const Transform& T);
    Transform toTransform() const;

    // Spherical Linear Interpolation
    static QuaternionPose slerp(const QuaternionPose& a,
                                 const QuaternionPose& b,
                                 double t);
};

} // namespace robot_controller::kinematics
```

### 3.3 IK Configuration Flags

```cpp
// IKConfiguration.hpp
#pragma once

#include <string>
#include <cstdint>

namespace robot_controller::kinematics {

// Configuration flags for 8 IK solutions
enum class ArmConfig : uint8_t {
    RIGHT = 0,  // Shoulder right (default)
    LEFT = 1    // Shoulder left
};

enum class ElbowConfig : uint8_t {
    UP = 0,     // Elbow up (default)
    DOWN = 1    // Elbow down
};

enum class WristConfig : uint8_t {
    NO_FLIP = 0,  // Wrist no flip (default)
    FLIP = 1      // Wrist flip (180° rotation)
};

struct IKConfiguration {
    ArmConfig arm = ArmConfig::RIGHT;
    ElbowConfig elbow = ElbowConfig::UP;
    WristConfig wrist = WristConfig::NO_FLIP;

    // Convert to index (0-7)
    uint8_t toIndex() const {
        return static_cast<uint8_t>(arm) * 4 +
               static_cast<uint8_t>(elbow) * 2 +
               static_cast<uint8_t>(wrist);
    }

    // Create from index
    static IKConfiguration fromIndex(uint8_t index) {
        IKConfiguration config;
        config.arm = static_cast<ArmConfig>((index >> 2) & 1);
        config.elbow = static_cast<ElbowConfig>((index >> 1) & 1);
        config.wrist = static_cast<WristConfig>(index & 1);
        return config;
    }

    // String representation
    std::string toString() const {
        return std::string(arm == ArmConfig::LEFT ? "L" : "R") +
               std::string(elbow == ElbowConfig::DOWN ? "D" : "U") +
               std::string(wrist == WristConfig::FLIP ? "F" : "N");
    }
};

// IK Solution with configuration
struct IKSolution {
    JointAngles joints;
    IKConfiguration config;
    bool is_valid = false;
    double fitness = 0.0;  // Lower is better (distance from current)
};

} // namespace robot_controller::kinematics
```

---

## 4. FORWARD KINEMATICS

### 4.1 Interface

```cpp
// ForwardKinematics.hpp
#pragma once

#include "DHParameters.hpp"
#include "Pose.hpp"

namespace robot_controller::kinematics {

class ForwardKinematics {
public:
    explicit ForwardKinematics(const DHParameterSet& dh_params);

    // Set TCP offset (tool frame relative to flange)
    void setTCPOffset(const Transform& tcp_offset);
    const Transform& getTCPOffset() const { return m_tcp_offset; }

    // Compute transform from base to TCP
    Transform compute(const JointAngles& q) const;

    // Compute transform from base to specific link (0 = base, 6 = flange)
    Transform computeToLink(const JointAngles& q, int link_index) const;

    // Compute all link transforms (for visualization)
    std::array<Transform, 7> computeAllLinks(const JointAngles& q) const;

    // Convert transform to CartesianPose
    CartesianPose transformToPose(const Transform& T) const;

    // Get pose directly from joint angles
    CartesianPose getPose(const JointAngles& q) const {
        return transformToPose(compute(q));
    }

private:
    // Compute single DH transform
    Transform dhTransform(double theta, const DHParameter& param) const;

    DHParameterSet m_dh_params;
    Transform m_tcp_offset = Transform::Identity();
};

} // namespace robot_controller::kinematics
```

### 4.2 Implementation

```cpp
// ForwardKinematics.cpp
#include "ForwardKinematics.hpp"
#include <cmath>

namespace robot_controller::kinematics {

ForwardKinematics::ForwardKinematics(const DHParameterSet& dh_params)
    : m_dh_params(dh_params) {}

void ForwardKinematics::setTCPOffset(const Transform& tcp_offset) {
    m_tcp_offset = tcp_offset;
}

Transform ForwardKinematics::dhTransform(double theta, const DHParameter& param) const {
    double ct = std::cos(theta + param.theta_offset);
    double st = std::sin(theta + param.theta_offset);
    double ca = std::cos(param.alpha);
    double sa = std::sin(param.alpha);
    double a = param.a;
    double d = param.d;

    Transform T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
          0,       sa,       ca,      d,
          0,        0,        0,      1;

    return T;
}

Transform ForwardKinematics::compute(const JointAngles& q) const {
    Transform T = Transform::Identity();

    for (int i = 0; i < 6; ++i) {
        T = T * dhTransform(q[i], m_dh_params[i]);
    }

    return T * m_tcp_offset;
}

Transform ForwardKinematics::computeToLink(const JointAngles& q, int link_index) const {
    if (link_index < 0 || link_index > 6) {
        return Transform::Identity();
    }

    Transform T = Transform::Identity();

    for (int i = 0; i < link_index; ++i) {
        T = T * dhTransform(q[i], m_dh_params[i]);
    }

    return T;
}

std::array<Transform, 7> ForwardKinematics::computeAllLinks(const JointAngles& q) const {
    std::array<Transform, 7> transforms;
    transforms[0] = Transform::Identity();  // Base

    for (int i = 0; i < 6; ++i) {
        transforms[i + 1] = transforms[i] * dhTransform(q[i], m_dh_params[i]);
    }

    return transforms;
}

CartesianPose ForwardKinematics::transformToPose(const Transform& T) const {
    CartesianPose pose;

    // Extract position
    pose.x = T(0, 3);
    pose.y = T(1, 3);
    pose.z = T(2, 3);

    // Extract rotation matrix
    RotationMatrix R = T.block<3, 3>(0, 0);

    // Convert to ZYX Euler angles (Rz * Ry * Rx)
    // ry = asin(-r31)
    // If cos(ry) ≈ 0 (gimbal lock):
    //   rx = 0, rz = atan2(-r12, r22)
    // Else:
    //   rx = atan2(r32, r33)
    //   rz = atan2(r21, r11)

    double r31 = R(2, 0);

    if (std::abs(r31) > 0.9999) {
        // Gimbal lock
        pose.ry = (r31 < 0) ? M_PI / 2.0 : -M_PI / 2.0;
        pose.rx = 0.0;
        pose.rz = std::atan2(-R(0, 1), R(1, 1));
    } else {
        pose.ry = -std::asin(r31);
        double cy = std::cos(pose.ry);
        pose.rx = std::atan2(R(2, 1) / cy, R(2, 2) / cy);
        pose.rz = std::atan2(R(1, 0) / cy, R(0, 0) / cy);
    }

    return pose;
}

} // namespace robot_controller::kinematics
```

---

## 5. INVERSE KINEMATICS

### 5.1 Analytical IK cho Robot PUMA-like

**Nguyên lý:**
Robot 6-DOF với spherical wrist có thể giải IK theo phương pháp giải tích bằng cách:
1. **Tách bài toán:** Position (Joint 1, 2, 3) và Orientation (Joint 4, 5, 6)
2. **Tính Wrist Center:** Từ target pose, trừ đi d6 theo hướng approach vector
3. **Giải J1, J2, J3:** Từ vị trí Wrist Center
4. **Giải J4, J5, J6:** Từ orientation còn lại

### 5.2 Interface

```cpp
// InverseKinematics.hpp
#pragma once

#include "ForwardKinematics.hpp"
#include "IKConfiguration.hpp"
#include <vector>
#include <optional>

namespace robot_controller::kinematics {

class InverseKinematics {
public:
    InverseKinematics(const DHParameterSet& dh_params,
                      const JointLimitSet& limits);

    // Set TCP offset (must match FK)
    void setTCPOffset(const Transform& tcp_offset);

    // Compute all valid IK solutions (up to 8)
    std::vector<IKSolution> computeAll(const Transform& target) const;

    // Compute solution with specific configuration
    std::optional<IKSolution> computeWithConfig(
        const Transform& target,
        const IKConfiguration& config) const;

    // Compute closest solution to current joint angles
    std::optional<IKSolution> computeClosest(
        const Transform& target,
        const JointAngles& current_q) const;

    // Check if pose is reachable
    bool isReachable(const Transform& target) const;

    // Check if solution is within joint limits
    bool isWithinLimits(const JointAngles& q) const;

    // Get current configuration from joint angles
    IKConfiguration getConfiguration(const JointAngles& q) const;

private:
    // Analytical solution steps
    Vector3 computeWristCenter(const Transform& target) const;

    // Solve for joints 1, 2, 3 (position)
    struct PositionSolution {
        double q1, q2, q3;
        bool valid;
    };
    std::array<PositionSolution, 4> solvePosition(const Vector3& wrist_center) const;

    // Solve for joints 4, 5, 6 (orientation)
    struct OrientationSolution {
        double q4, q5, q6;
        bool valid;
    };
    std::array<OrientationSolution, 2> solveOrientation(
        const JointAngles& q123,
        const RotationMatrix& target_R) const;

    // Angle utilities
    double normalizeAngle(double angle) const;
    double atan2Safe(double y, double x) const;

    DHParameterSet m_dh_params;
    JointLimitSet m_limits;
    Transform m_tcp_offset = Transform::Identity();

    // Derived parameters for analytical solution
    double m_d1, m_a2, m_a3, m_d4, m_d6;
    double m_shoulder_offset;  // d2 or similar
};

} // namespace robot_controller::kinematics
```

### 5.3 Implementation (Core Algorithm)

```cpp
// InverseKinematics.cpp
#include "InverseKinematics.hpp"
#include <cmath>
#include <algorithm>

namespace robot_controller::kinematics {

InverseKinematics::InverseKinematics(const DHParameterSet& dh_params,
                                      const JointLimitSet& limits)
    : m_dh_params(dh_params), m_limits(limits) {

    // Extract key dimensions from DH parameters
    m_d1 = dh_params[0].d;
    m_a2 = dh_params[1].a;
    m_a3 = dh_params[2].a;
    m_d4 = dh_params[3].d;
    m_d6 = dh_params[5].d;
    m_shoulder_offset = dh_params[1].d;  // d2
}

Vector3 InverseKinematics::computeWristCenter(const Transform& target) const {
    // Wrist center = TCP position - d6 * approach_vector
    Vector3 tcp_pos = target.block<3, 1>(0, 3);
    Vector3 approach = target.block<3, 1>(0, 2);  // Z-axis of target frame

    // Adjust for TCP offset if set
    if (!m_tcp_offset.isIdentity()) {
        // Transform back to flange
        Transform T_flange = target * m_tcp_offset.inverse();
        tcp_pos = T_flange.block<3, 1>(0, 3);
        approach = T_flange.block<3, 1>(0, 2);
    }

    return tcp_pos - m_d6 * approach;
}

std::array<InverseKinematics::PositionSolution, 4>
InverseKinematics::solvePosition(const Vector3& wc) const {
    std::array<PositionSolution, 4> solutions;
    for (auto& sol : solutions) sol.valid = false;

    double wx = wc.x();
    double wy = wc.y();
    double wz = wc.z();

    // ===== Joint 1 (2 solutions: ARM LEFT/RIGHT) =====
    // q1 = atan2(wy, wx) or q1 = atan2(-wy, -wx)

    double r_xy = std::sqrt(wx * wx + wy * wy);

    // Check reachability in XY plane
    if (r_xy < 1e-6) {
        // Singularity: wrist center on Z-axis
        // Any q1 works, use 0
        solutions[0].q1 = 0;
        solutions[1].q1 = 0;
        solutions[2].q1 = M_PI;
        solutions[3].q1 = M_PI;
    } else {
        // ARM RIGHT
        double q1_right = std::atan2(wy, wx);
        // ARM LEFT (offset by pi, adjusted for shoulder)
        double q1_left = std::atan2(-wy, -wx);

        // Account for shoulder offset if present
        if (std::abs(m_shoulder_offset) > 1e-6) {
            double offset_angle = std::asin(m_shoulder_offset / r_xy);
            q1_right -= offset_angle;
            q1_left += offset_angle;
        }

        solutions[0].q1 = q1_right;
        solutions[1].q1 = q1_right;
        solutions[2].q1 = q1_left;
        solutions[3].q1 = q1_left;
    }

    // ===== Joint 2 and 3 (2 solutions each: ELBOW UP/DOWN) =====
    for (int arm = 0; arm < 2; ++arm) {
        double q1 = solutions[arm * 2].q1;

        // Project wrist center onto arm plane
        double c1 = std::cos(q1);
        double s1 = std::sin(q1);

        // Distance from shoulder to wrist center in arm plane
        double px = c1 * wx + s1 * wy - m_shoulder_offset;
        double pz = wz - m_d1;

        double r_sq = px * px + pz * pz;
        double r = std::sqrt(r_sq);

        // Check reachability (triangle inequality)
        double L1 = m_a2;  // Upper arm length
        double L2 = std::sqrt(m_a3 * m_a3 + m_d4 * m_d4);  // Forearm + wrist

        if (r > L1 + L2 || r < std::abs(L1 - L2)) {
            // Unreachable
            continue;
        }

        // Law of cosines for elbow angle
        double cos_q3 = (r_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        cos_q3 = std::clamp(cos_q3, -1.0, 1.0);

        double q3_offset = std::atan2(m_d4, std::abs(m_a3));

        // ELBOW UP
        double q3_up = std::acos(cos_q3) - q3_offset;
        // ELBOW DOWN
        double q3_down = -std::acos(cos_q3) - q3_offset;

        // Solve for q2
        auto solve_q2 = [&](double q3) -> double {
            double s3 = std::sin(q3 + q3_offset);
            double c3 = std::cos(q3 + q3_offset);

            double k1 = L1 + L2 * c3;
            double k2 = L2 * s3;

            return std::atan2(pz, px) - std::atan2(k2, k1);
        };

        int idx_up = arm * 2;
        int idx_down = arm * 2 + 1;

        solutions[idx_up].q2 = solve_q2(q3_up);
        solutions[idx_up].q3 = q3_up;
        solutions[idx_up].valid = true;

        solutions[idx_down].q2 = solve_q2(q3_down);
        solutions[idx_down].q3 = q3_down;
        solutions[idx_down].valid = true;
    }

    return solutions;
}

std::array<InverseKinematics::OrientationSolution, 2>
InverseKinematics::solveOrientation(const JointAngles& q123,
                                     const RotationMatrix& target_R) const {
    std::array<OrientationSolution, 2> solutions;
    for (auto& sol : solutions) sol.valid = false;

    // Compute R0_3 (rotation from base to joint 3)
    ForwardKinematics fk(m_dh_params);
    JointAngles q_temp = {q123[0], q123[1], q123[2], 0, 0, 0};
    Transform T0_3 = fk.computeToLink(q_temp, 3);
    RotationMatrix R0_3 = T0_3.block<3, 3>(0, 0);

    // R3_6 = R0_3^T * R0_6
    RotationMatrix R3_6 = R0_3.transpose() * target_R;

    // Extract Euler angles ZYZ for spherical wrist
    // R3_6 = Rz(q4) * Ry(q5) * Rz(q6)

    double r33 = R3_6(2, 2);
    double r13 = R3_6(0, 2);
    double r23 = R3_6(1, 2);
    double r31 = R3_6(2, 0);
    double r32 = R3_6(2, 1);

    // WRIST NO FLIP
    if (std::abs(r33) < 0.9999) {
        double q5 = std::acos(r33);

        solutions[0].q4 = std::atan2(r23, r13);
        solutions[0].q5 = q5;
        solutions[0].q6 = std::atan2(r32, -r31);
        solutions[0].valid = true;

        // WRIST FLIP (q5 negative)
        solutions[1].q4 = std::atan2(-r23, -r13);
        solutions[1].q5 = -q5;
        solutions[1].q6 = std::atan2(-r32, r31);
        solutions[1].valid = true;
    } else {
        // Singularity: q5 = 0 or π
        // q4 and q6 are coupled
        if (r33 > 0) {
            // q5 ≈ 0
            solutions[0].q5 = 0;
            solutions[0].q4 = 0;  // Arbitrary
            solutions[0].q6 = std::atan2(R3_6(1, 0), R3_6(0, 0));
            solutions[0].valid = true;
        } else {
            // q5 ≈ π
            solutions[0].q5 = M_PI;
            solutions[0].q4 = 0;
            solutions[0].q6 = std::atan2(-R3_6(1, 0), -R3_6(0, 0));
            solutions[0].valid = true;
        }
    }

    return solutions;
}

std::vector<IKSolution> InverseKinematics::computeAll(const Transform& target) const {
    std::vector<IKSolution> valid_solutions;

    // Step 1: Compute wrist center
    Vector3 wc = computeWristCenter(target);

    // Step 2: Solve for q1, q2, q3 (4 solutions)
    auto pos_solutions = solvePosition(wc);

    // Step 3: For each position solution, solve for q4, q5, q6 (2 solutions each)
    RotationMatrix target_R = target.block<3, 3>(0, 0);

    for (int i = 0; i < 4; ++i) {
        if (!pos_solutions[i].valid) continue;

        JointAngles q123 = {pos_solutions[i].q1, pos_solutions[i].q2,
                           pos_solutions[i].q3, 0, 0, 0};

        auto orient_solutions = solveOrientation(q123, target_R);

        for (int j = 0; j < 2; ++j) {
            if (!orient_solutions[j].valid) continue;

            IKSolution sol;
            sol.joints = {
                normalizeAngle(pos_solutions[i].q1),
                normalizeAngle(pos_solutions[i].q2),
                normalizeAngle(pos_solutions[i].q3),
                normalizeAngle(orient_solutions[j].q4),
                normalizeAngle(orient_solutions[j].q5),
                normalizeAngle(orient_solutions[j].q6)
            };

            // Check joint limits
            if (isWithinLimits(sol.joints)) {
                sol.config = IKConfiguration::fromIndex(i * 2 + j);
                sol.is_valid = true;
                valid_solutions.push_back(sol);
            }
        }
    }

    return valid_solutions;
}

std::optional<IKSolution> InverseKinematics::computeClosest(
    const Transform& target,
    const JointAngles& current_q) const {

    auto solutions = computeAll(target);

    if (solutions.empty()) {
        return std::nullopt;
    }

    // Find solution with minimum weighted joint movement
    const std::array<double, 6> weights = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};

    double min_cost = std::numeric_limits<double>::max();
    IKSolution* best = nullptr;

    for (auto& sol : solutions) {
        double cost = 0;
        for (int i = 0; i < 6; ++i) {
            double diff = normalizeAngle(sol.joints[i] - current_q[i]);
            cost += weights[i] * diff * diff;
        }
        sol.fitness = cost;

        if (cost < min_cost) {
            min_cost = cost;
            best = &sol;
        }
    }

    return *best;
}

bool InverseKinematics::isWithinLimits(const JointAngles& q) const {
    for (int i = 0; i < 6; ++i) {
        if (!m_limits[i].isWithinLimit(q[i])) {
            return false;
        }
    }
    return true;
}

double InverseKinematics::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace robot_controller::kinematics
```

---

## 6. JACOBIAN

### 6.1 Lý thuyết

Jacobian matrix J ánh xạ vận tốc từ không gian khớp sang không gian Cartesian:

```
ẋ = J(q) × q̇

Trong đó:
- ẋ = [vx, vy, vz, ωx, ωy, ωz]^T  (Cartesian velocity, 6×1)
- q̇ = [q̇1, q̇2, ..., q̇6]^T      (Joint velocity, 6×1)
- J = 6×6 Jacobian matrix
```

### 6.2 Interface

```cpp
// JacobianCalculator.hpp
#pragma once

#include "ForwardKinematics.hpp"

namespace robot_controller::kinematics {

// 6x6 Jacobian matrix
using Jacobian = Eigen::Matrix<double, 6, 6>;

class JacobianCalculator {
public:
    explicit JacobianCalculator(const ForwardKinematics& fk);

    // Compute geometric Jacobian at given configuration
    Jacobian compute(const JointAngles& q) const;

    // Compute Jacobian pseudo-inverse (for singularity handling)
    Jacobian computePseudoInverse(const JointAngles& q,
                                   double damping = 0.01) const;

    // Check for singularity
    bool isSingular(const JointAngles& q, double threshold = 0.001) const;

    // Get manipulability measure (sqrt(det(J*J^T)))
    double getManipulability(const JointAngles& q) const;

    // Get condition number (measure of singularity proximity)
    double getConditionNumber(const JointAngles& q) const;

private:
    const ForwardKinematics& m_fk;
};

} // namespace robot_controller::kinematics
```

### 6.3 Implementation

```cpp
// JacobianCalculator.cpp
#include "JacobianCalculator.hpp"
#include <Eigen/SVD>

namespace robot_controller::kinematics {

JacobianCalculator::JacobianCalculator(const ForwardKinematics& fk)
    : m_fk(fk) {}

Jacobian JacobianCalculator::compute(const JointAngles& q) const {
    Jacobian J = Jacobian::Zero();

    // Compute all link transforms
    auto transforms = m_fk.computeAllLinks(q);

    // End-effector position
    Vector3 p_e = transforms[6].block<3, 1>(0, 3);

    for (int i = 0; i < 6; ++i) {
        // Z-axis of joint i frame
        Vector3 z_i = transforms[i].block<3, 1>(0, 2);

        // Position of joint i origin
        Vector3 p_i = transforms[i].block<3, 1>(0, 3);

        // For revolute joints:
        // Linear velocity contribution: z_i × (p_e - p_i)
        // Angular velocity contribution: z_i

        Vector3 linear = z_i.cross(p_e - p_i);
        Vector3 angular = z_i;

        J.block<3, 1>(0, i) = linear;
        J.block<3, 1>(3, i) = angular;
    }

    return J;
}

Jacobian JacobianCalculator::computePseudoInverse(const JointAngles& q,
                                                    double damping) const {
    Jacobian J = compute(q);

    // Damped least squares (DLS) pseudo-inverse
    // J^+ = J^T * (J * J^T + λ²I)^-1

    Jacobian JJt = J * J.transpose();
    Jacobian JJt_damped = JJt + damping * damping * Jacobian::Identity();

    return J.transpose() * JJt_damped.inverse();
}

bool JacobianCalculator::isSingular(const JointAngles& q, double threshold) const {
    return getManipulability(q) < threshold;
}

double JacobianCalculator::getManipulability(const JointAngles& q) const {
    Jacobian J = compute(q);

    // Manipulability = sqrt(det(J * J^T))
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    return std::sqrt(JJt.determinant());
}

double JacobianCalculator::getConditionNumber(const JointAngles& q) const {
    Jacobian J = compute(q);

    // Condition number = σ_max / σ_min
    Eigen::JacobiSVD<Jacobian> svd(J);
    auto singular_values = svd.singularValues();

    if (singular_values(5) < 1e-10) {
        return std::numeric_limits<double>::infinity();
    }

    return singular_values(0) / singular_values(5);
}

} // namespace robot_controller::kinematics
```

---

## 7. KINEMATICS ENGINE (Unified Interface)

### 7.1 Interface

```cpp
// KinematicsEngine.hpp
#pragma once

#include "ForwardKinematics.hpp"
#include "InverseKinematics.hpp"
#include "JacobianCalculator.hpp"
#include <memory>

namespace robot_controller::kinematics {

class KinematicsEngine {
public:
    KinematicsEngine(const DHParameterSet& dh_params,
                     const JointLimitSet& limits);

    // Configuration
    void setTCPOffset(const Transform& offset);
    void setPreferredConfig(const IKConfiguration& config);

    // Forward Kinematics
    Transform computeFK(const JointAngles& q) const;
    CartesianPose getPose(const JointAngles& q) const;
    std::array<Transform, 7> getAllLinkTransforms(const JointAngles& q) const;

    // Inverse Kinematics
    std::optional<JointAngles> computeIK(
        const Transform& target,
        const JointAngles& current_q) const;

    std::optional<JointAngles> computeIK(
        const CartesianPose& target,
        const JointAngles& current_q) const;

    std::vector<IKSolution> getAllIKSolutions(const Transform& target) const;

    // Jacobian
    Jacobian computeJacobian(const JointAngles& q) const;
    bool isSingular(const JointAngles& q) const;
    double getManipulability(const JointAngles& q) const;

    // Velocity conversion
    Vector6 jointToCartesianVelocity(const JointAngles& q,
                                      const JointAngles& q_dot) const;
    JointAngles cartesianToJointVelocity(const JointAngles& q,
                                          const Vector6& cart_vel) const;

    // Validation
    bool isReachable(const Transform& target) const;
    bool isWithinLimits(const JointAngles& q) const;
    IKConfiguration getCurrentConfig(const JointAngles& q) const;

    // Utility
    double jointDistance(const JointAngles& a, const JointAngles& b) const;

private:
    std::unique_ptr<ForwardKinematics> m_fk;
    std::unique_ptr<InverseKinematics> m_ik;
    std::unique_ptr<JacobianCalculator> m_jacobian;

    IKConfiguration m_preferred_config;
    JointLimitSet m_limits;
};

} // namespace robot_controller::kinematics
```

### 7.2 Implementation

```cpp
// KinematicsEngine.cpp
#include "KinematicsEngine.hpp"

namespace robot_controller::kinematics {

KinematicsEngine::KinematicsEngine(const DHParameterSet& dh_params,
                                    const JointLimitSet& limits)
    : m_limits(limits) {

    m_fk = std::make_unique<ForwardKinematics>(dh_params);
    m_ik = std::make_unique<InverseKinematics>(dh_params, limits);
    m_jacobian = std::make_unique<JacobianCalculator>(*m_fk);
}

void KinematicsEngine::setTCPOffset(const Transform& offset) {
    m_fk->setTCPOffset(offset);
    m_ik->setTCPOffset(offset);
}

Transform KinematicsEngine::computeFK(const JointAngles& q) const {
    return m_fk->compute(q);
}

CartesianPose KinematicsEngine::getPose(const JointAngles& q) const {
    return m_fk->getPose(q);
}

std::optional<JointAngles> KinematicsEngine::computeIK(
    const Transform& target,
    const JointAngles& current_q) const {

    auto solution = m_ik->computeClosest(target, current_q);

    if (solution && solution->is_valid) {
        return solution->joints;
    }

    return std::nullopt;
}

std::optional<JointAngles> KinematicsEngine::computeIK(
    const CartesianPose& target,
    const JointAngles& current_q) const {

    return computeIK(target.toTransform(), current_q);
}

JointAngles KinematicsEngine::cartesianToJointVelocity(
    const JointAngles& q,
    const Vector6& cart_vel) const {

    // q̇ = J^-1 * ẋ (using damped pseudo-inverse)
    Jacobian J_inv = m_jacobian->computePseudoInverse(q, 0.01);

    Eigen::Matrix<double, 6, 1> q_dot = J_inv * cart_vel;

    JointAngles result;
    for (int i = 0; i < 6; ++i) {
        result[i] = q_dot(i);
    }

    return result;
}

double KinematicsEngine::jointDistance(const JointAngles& a,
                                        const JointAngles& b) const {
    double sum = 0;
    for (int i = 0; i < 6; ++i) {
        double diff = a[i] - b[i];
        // Normalize to [-π, π]
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

} // namespace robot_controller::kinematics
```

---

## 8. TÍCH HỢP VỚI ROBOTICS LIBRARY (Optional)

### 8.1 Khi nào sử dụng Robotics Library

| Trường hợp | Giải pháp |
|------------|-----------|
| Robot PUMA-like đơn giản | Custom implementation (như trên) |
| Robot phức tạp (offset wrist) | Robotics Library `rl::kin::Puma` |
| Cần collision checking | Robotics Library `rl::sg` |
| Cần path planning | Robotics Library `rl::plan` |

### 8.2 Wrapper cho Robotics Library

```cpp
// RoboticsLibraryAdapter.hpp
#pragma once

#ifdef USE_ROBOTICS_LIBRARY

#include "KinematicsEngine.hpp"
#include <rl/kin/Puma.h>
#include <rl/mdl/XmlFactory.h>

namespace robot_controller::kinematics {

class RoboticsLibraryAdapter : public KinematicsEngine {
public:
    explicit RoboticsLibraryAdapter(const std::string& model_path);

    // Override IK to use RL's analytical solver
    std::optional<JointAngles> computeIK(
        const Transform& target,
        const JointAngles& current_q) const override;

    std::vector<IKSolution> getAllIKSolutions(
        const Transform& target) const override;

private:
    std::unique_ptr<rl::kin::Puma> m_rl_kinematics;
};

} // namespace robot_controller::kinematics

#endif // USE_ROBOTICS_LIBRARY
```

---

## 9. TESTING

### 9.1 Unit Test Cases

```cpp
// test_Kinematics.cpp
#include <gtest/gtest.h>
#include "KinematicsEngine.hpp"

using namespace robot_controller::kinematics;

class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // PUMA 560 DH parameters
        DHParameterSet dh = {{
            {0, 0, 0, -M_PI_2},
            {0, 149.09, 431.80, 0},
            {0, 0, -20.32, M_PI_2},
            {0, 433.07, 0, -M_PI_2},
            {0, 0, 0, M_PI_2},
            {0, 56.25, 0, 0}
        }};

        JointLimitSet limits = {{
            {-2.79, 2.79},
            {-3.93, 0.79},
            {-0.79, 3.93},
            {-5.24, 5.24},
            {-2.09, 2.09},
            {-6.28, 6.28}
        }};

        engine = std::make_unique<KinematicsEngine>(dh, limits);
    }

    std::unique_ptr<KinematicsEngine> engine;
};

TEST_F(KinematicsTest, FK_HomePosition) {
    JointAngles home = {0, 0, 0, 0, 0, 0};
    CartesianPose pose = engine->getPose(home);

    // Expected position at home (rough values)
    EXPECT_NEAR(pose.x, 411.48, 1.0);
    EXPECT_NEAR(pose.y, 0, 0.1);
    EXPECT_NEAR(pose.z, 582.16, 1.0);
}

TEST_F(KinematicsTest, IK_RoundTrip) {
    // Generate random configuration
    JointAngles original = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};

    // FK to get pose
    Transform T = engine->computeFK(original);

    // IK to get joints back
    auto result = engine->computeIK(T, original);

    ASSERT_TRUE(result.has_value());

    // Check joint angles match
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(result.value()[i], original[i], 0.001);
    }
}

TEST_F(KinematicsTest, IK_AllSolutions) {
    JointAngles q = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};
    Transform T = engine->computeFK(q);

    auto solutions = engine->getAllIKSolutions(T);

    // Should have up to 8 solutions
    EXPECT_GT(solutions.size(), 0);
    EXPECT_LE(solutions.size(), 8);

    // All solutions should produce the same pose
    for (const auto& sol : solutions) {
        Transform T_check = engine->computeFK(sol.joints);

        // Position error
        double pos_error = (T.block<3,1>(0,3) - T_check.block<3,1>(0,3)).norm();
        EXPECT_LT(pos_error, 0.01);
    }
}

TEST_F(KinematicsTest, Jacobian_Singularity) {
    // At singularity (q5 = 0), robot should report singular
    JointAngles singular = {0, 0, 0, 0, 0, 0};
    EXPECT_TRUE(engine->isSingular(singular));

    // Away from singularity
    JointAngles non_singular = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};
    EXPECT_FALSE(engine->isSingular(non_singular));
}

TEST_F(KinematicsTest, VelocityMapping) {
    JointAngles q = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};
    JointAngles q_dot = {0.1, -0.05, 0.08, 0.02, -0.05, 0.1};

    // Joint to Cartesian
    Vector6 cart_vel = engine->jointToCartesianVelocity(q, q_dot);

    // Back to joint (should be close to original)
    JointAngles q_dot_back = engine->cartesianToJointVelocity(q, cart_vel);

    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(q_dot_back[i], q_dot[i], 0.001);
    }
}
```

### 9.2 Performance Benchmarks

```cpp
// benchmark_Kinematics.cpp
#include <benchmark/benchmark.h>
#include "KinematicsEngine.hpp"

static void BM_ForwardKinematics(benchmark::State& state) {
    auto engine = createTestEngine();
    JointAngles q = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};

    for (auto _ : state) {
        benchmark::DoNotOptimize(engine->computeFK(q));
    }
}
BENCHMARK(BM_ForwardKinematics);

static void BM_InverseKinematics(benchmark::State& state) {
    auto engine = createTestEngine();
    JointAngles q = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};
    Transform T = engine->computeFK(q);

    for (auto _ : state) {
        benchmark::DoNotOptimize(engine->computeIK(T, q));
    }
}
BENCHMARK(BM_InverseKinematics);

static void BM_Jacobian(benchmark::State& state) {
    auto engine = createTestEngine();
    JointAngles q = {0.5, -0.3, 0.8, 0.2, -0.5, 1.0};

    for (auto _ : state) {
        benchmark::DoNotOptimize(engine->computeJacobian(q));
    }
}
BENCHMARK(BM_Jacobian);

BENCHMARK_MAIN();
```

**Expected Results:**
| Operation | Target | Typical |
|-----------|--------|---------|
| FK | < 10μs | ~2-5μs |
| IK (all solutions) | < 50μs | ~20-30μs |
| IK (closest) | < 60μs | ~25-40μs |
| Jacobian | < 20μs | ~5-10μs |

---

## 10. CONFIGURATION & USAGE

### 10.1 Robot Configuration File

```yaml
# robot_config.yaml
robot:
  name: "WeldingRobot_6DOF"
  type: "PUMA_LIKE"

  dh_parameters:
    # Joint 1
    - theta_offset: 0.0
      d: 0.0
      a: 0.0
      alpha: -1.5707963  # -π/2

    # Joint 2
    - theta_offset: 0.0
      d: 149.09
      a: 431.80
      alpha: 0.0

    # Joint 3
    - theta_offset: 0.0
      d: 0.0
      a: -20.32
      alpha: 1.5707963   # π/2

    # Joint 4
    - theta_offset: 0.0
      d: 433.07
      a: 0.0
      alpha: -1.5707963  # -π/2

    # Joint 5
    - theta_offset: 0.0
      d: 0.0
      a: 0.0
      alpha: 1.5707963   # π/2

    # Joint 6
    - theta_offset: 0.0
      d: 56.25
      a: 0.0
      alpha: 0.0

  joint_limits:
    - min: -160.0  # degrees
      max: 160.0
    - min: -225.0
      max: 45.0
    - min: -45.0
      max: 225.0
    - min: -300.0
      max: 300.0
    - min: -120.0
      max: 120.0
    - min: -360.0
      max: 360.0

  tcp_offset:
    x: 0.0
    y: 0.0
    z: 100.0  # Tool length
    rx: 0.0
    ry: 0.0
    rz: 0.0
```

### 10.2 Usage Example

```cpp
#include "KinematicsEngine.hpp"
#include "ConfigLoader.hpp"

int main() {
    // Load configuration
    auto config = ConfigLoader::load("robot_config.yaml");

    // Create kinematics engine
    KinematicsEngine kinematics(config.dh_params, config.joint_limits);
    kinematics.setTCPOffset(config.tcp_offset);

    // Current joint angles
    JointAngles current = {0, -M_PI/4, M_PI/2, 0, M_PI/4, 0};

    // Get current pose
    CartesianPose pose = kinematics.getPose(current);
    std::cout << "Position: " << pose.x << ", " << pose.y << ", " << pose.z << std::endl;

    // Target pose (move 100mm in X)
    pose.x += 100;

    // Compute IK
    auto target_joints = kinematics.computeIK(pose, current);

    if (target_joints) {
        std::cout << "Target joints: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << target_joints.value()[i] * 180 / M_PI << "° ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Target pose is unreachable!" << std::endl;
    }

    return 0;
}
```

---

## 11. DEPENDENCIES

### 11.1 Required Libraries

| Library | Version | Purpose |
|---------|---------|---------|
| Eigen3 | 3.4+ | Linear algebra |
| (Optional) Robotics Library | 0.7+ | Advanced IK, collision |

### 11.2 CMake Configuration

```cmake
# CMakeLists.txt for kinematics module

find_package(Eigen3 3.4 REQUIRED)

add_library(kinematics
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    src/JacobianCalculator.cpp
    src/KinematicsEngine.cpp
    src/Pose.cpp
)

target_include_directories(kinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kinematics PUBLIC
    Eigen3::Eigen
)

# Optional: Robotics Library
option(USE_ROBOTICS_LIBRARY "Use Robotics Library for advanced IK" OFF)
if(USE_ROBOTICS_LIBRARY)
    find_package(RL REQUIRED)
    target_link_libraries(kinematics PRIVATE RL::kin RL::mdl)
    target_compile_definitions(kinematics PRIVATE USE_ROBOTICS_LIBRARY)
endif()

# Tests
if(BUILD_TESTING)
    add_executable(test_kinematics tests/test_Kinematics.cpp)
    target_link_libraries(test_kinematics kinematics GTest::gtest_main)
    add_test(NAME test_kinematics COMMAND test_kinematics)
endif()
```

---

## 12. REVISION HISTORY

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 01/02/2026 | - | Initial draft |

---

*Document generated following project roadmap structure.*
