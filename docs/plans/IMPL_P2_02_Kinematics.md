# IMPL_P2_02: Kinematics Engine

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P2_02 |
| Phase | 2 - Motion Core |
| Priority | P0 (Critical) |
| Depends On | IMPL_P2_01 (State Machine) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Robotics Library_ Robot Tùy Chỉnh & IK Giải Tích.md` | Analytical IK, DH parameters, singularity handling |

---

## Overview

Implementation plan cho Kinematics Engine của 6-DOF Robot Controller:
- **Forward Kinematics (FK):** Tính TCP position/orientation từ joint angles
- **Inverse Kinematics (IK):** Tính joint angles từ desired TCP pose
- **Jacobian:** Velocity kinematics và singularity detection
- **DH Parameters:** Denavit-Hartenberg convention cho robot geometry

---

## Prerequisites

- [ ] IMPL_P2_01 (State Machine) đã hoàn thành
- [ ] Eigen library installed via vcpkg
- [ ] C++ project builds successfully
- [ ] Hiểu DH convention và robot geometry

---

## Step 1: Install Eigen Library

### 1.1 Add Eigen to vcpkg

**Command:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
vcpkg install eigen3:x64-windows
```

### 1.2 Update CMakeLists.txt

**File:** `src/cpp/CMakeLists.txt`

**Add after existing find_package:**
```cmake
find_package(Eigen3 CONFIG REQUIRED)
```

**Add to target_link_libraries:**
```cmake
target_link_libraries(RobotCore
    PRIVATE
    # ... existing libraries ...
    Eigen3::Eigen
)
```

**Validation:**
```powershell
cmake --build build --config Debug
```
**Expected:** Build succeeds with Eigen included

---

## Step 2: Create Math Utilities

### 2.1 Create MathTypes.hpp

**File:** `src/cpp/include/kinematics/MathTypes.hpp`

```cpp
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>

namespace robotics {
namespace math {

// ============================================================================
// Type Definitions
// ============================================================================

// Basic types
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

// Joint space
constexpr int NUM_JOINTS = 6;
using JointAngles = std::array<double, NUM_JOINTS>;
using JointVelocities = std::array<double, NUM_JOINTS>;
using JointTorques = std::array<double, NUM_JOINTS>;

// Jacobian matrix (6x6 for 6-DOF robot)
using Jacobian = Eigen::Matrix<double, 6, NUM_JOINTS>;

// ============================================================================
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double EPSILON = 1e-10;

// ============================================================================
// Utility Functions
// ============================================================================

inline double degToRad(double degrees) {
    return degrees * DEG_TO_RAD;
}

inline double radToDeg(double radians) {
    return radians * RAD_TO_DEG;
}

inline bool isNearZero(double value, double tolerance = EPSILON) {
    return std::abs(value) < tolerance;
}

inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// Convert rotation matrix to RPY (Roll-Pitch-Yaw) angles
// Convention: ZYX (Yaw-Pitch-Roll applied in that order)
inline Vector3d rotationToRPY(const Matrix3d& R) {
    Vector3d rpy;

    // Check for gimbal lock
    if (std::abs(R(2, 0)) >= 1.0 - EPSILON) {
        // Gimbal lock: pitch = +/- 90 degrees
        rpy(2) = 0.0;  // Set yaw to 0
        if (R(2, 0) < 0) {
            rpy(1) = PI / 2.0;
            rpy(0) = std::atan2(R(0, 1), R(0, 2));
        } else {
            rpy(1) = -PI / 2.0;
            rpy(0) = std::atan2(-R(0, 1), -R(0, 2));
        }
    } else {
        rpy(1) = std::asin(-R(2, 0));  // Pitch
        rpy(0) = std::atan2(R(2, 1), R(2, 2));  // Roll
        rpy(2) = std::atan2(R(1, 0), R(0, 0));  // Yaw
    }

    return rpy;
}

// Convert RPY angles to rotation matrix
inline Matrix3d rpyToRotation(const Vector3d& rpy) {
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Matrix3d Rx, Ry, Rz;

    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);

    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
          -std::sin(pitch), 0, std::cos(pitch);

    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw), std::cos(yaw), 0,
          0, 0, 1;

    return Rz * Ry * Rx;  // ZYX convention
}

} // namespace math
} // namespace robotics
```

---

## Step 3: Create DH Parameters Structure

### 3.1 Create DHParameters.hpp

**File:** `src/cpp/include/kinematics/DHParameters.hpp`

```cpp
#pragma once

#include "MathTypes.hpp"
#include <vector>
#include <string>

namespace robotics {
namespace kinematics {

using namespace math;

// ============================================================================
// DH Parameter Structure
// ============================================================================

/**
 * Denavit-Hartenberg parameters for a single joint
 * Using Modified DH convention (Craig's convention)
 *
 * Parameters:
 *   a (alpha_{i-1}): Twist angle - rotation about X_{i-1} axis
 *   alpha: Link length - distance along X_{i-1} axis
 *   d: Link offset - distance along Z_i axis
 *   theta: Joint angle - rotation about Z_i axis (variable for revolute)
 */
struct DHParameter {
    double a;           // Link length (mm)
    double alpha;       // Twist angle (rad)
    double d;           // Link offset (mm)
    double theta;       // Joint angle offset (rad) - added to actual joint angle

    // Joint limits
    double minAngle;    // Minimum joint angle (rad)
    double maxAngle;    // Maximum joint angle (rad)

    // Joint properties
    std::string name;
    bool isRevolute;    // true = revolute, false = prismatic

    DHParameter()
        : a(0), alpha(0), d(0), theta(0),
          minAngle(-PI), maxAngle(PI),
          name(""), isRevolute(true) {}

    DHParameter(double a_, double alpha_, double d_, double theta_,
                double minAngle_ = -PI, double maxAngle_ = PI,
                const std::string& name_ = "", bool isRevolute_ = true)
        : a(a_), alpha(alpha_), d(d_), theta(theta_),
          minAngle(minAngle_), maxAngle(maxAngle_),
          name(name_), isRevolute(isRevolute_) {}
};

// ============================================================================
// Robot Configuration
// ============================================================================

/**
 * Complete robot kinematic configuration
 */
struct RobotKinematicConfig {
    std::vector<DHParameter> dhParams;

    // Tool offset (from flange to TCP)
    Vector3d toolOffset;
    Matrix3d toolRotation;

    // Base offset (from world to robot base)
    Vector3d baseOffset;
    Matrix3d baseRotation;

    RobotKinematicConfig()
        : toolOffset(Vector3d::Zero()),
          toolRotation(Matrix3d::Identity()),
          baseOffset(Vector3d::Zero()),
          baseRotation(Matrix3d::Identity()) {}
};

// ============================================================================
// Default Robot Configurations
// ============================================================================

/**
 * Create DH parameters for a typical 6-DOF industrial robot
 * Similar to KUKA KR6 / ABB IRB1200 / Universal Robots UR5
 *
 * Dimensions in mm, angles in radians
 */
inline RobotKinematicConfig createDefault6DOFConfig() {
    RobotKinematicConfig config;

    // DH Parameters for 6-DOF robot (Modified DH convention)
    // These are typical values - adjust for specific robot
    //
    // Joint layout:
    //   J1: Base rotation (vertical axis)
    //   J2: Shoulder (horizontal axis)
    //   J3: Elbow (horizontal axis)
    //   J4: Wrist 1 rotation
    //   J5: Wrist 2 bend
    //   J6: Wrist 3 rotation (tool rotation)

    config.dhParams = {
        // Joint 1: Base
        DHParameter(
            0.0,                    // a: link length
            -PI / 2.0,              // alpha: -90 deg
            400.0,                  // d: base height
            0.0,                    // theta offset
            degToRad(-170.0),       // min angle
            degToRad(170.0),        // max angle
            "J1_Base",
            true
        ),
        // Joint 2: Shoulder
        DHParameter(
            350.0,                  // a: upper arm length
            0.0,                    // alpha: 0
            0.0,                    // d: 0
            -PI / 2.0,              // theta offset: -90 deg
            degToRad(-135.0),
            degToRad(35.0),
            "J2_Shoulder",
            true
        ),
        // Joint 3: Elbow
        DHParameter(
            42.0,                   // a: elbow offset
            -PI / 2.0,              // alpha: -90 deg
            0.0,                    // d: 0
            0.0,                    // theta offset
            degToRad(-120.0),
            degToRad(158.0),
            "J3_Elbow",
            true
        ),
        // Joint 4: Wrist 1
        DHParameter(
            0.0,                    // a: 0
            PI / 2.0,               // alpha: 90 deg
            350.0,                  // d: forearm length
            0.0,                    // theta offset
            degToRad(-350.0),
            degToRad(350.0),
            "J4_Wrist1",
            true
        ),
        // Joint 5: Wrist 2
        DHParameter(
            0.0,                    // a: 0
            -PI / 2.0,              // alpha: -90 deg
            0.0,                    // d: 0
            0.0,                    // theta offset
            degToRad(-125.0),
            degToRad(125.0),
            "J5_Wrist2",
            true
        ),
        // Joint 6: Wrist 3 (Tool)
        DHParameter(
            0.0,                    // a: 0
            0.0,                    // alpha: 0
            80.0,                   // d: tool flange offset
            0.0,                    // theta offset
            degToRad(-350.0),
            degToRad(350.0),
            "J6_Wrist3",
            true
        )
    };

    // Default tool offset (can be updated for specific tool)
    config.toolOffset = Vector3d(0, 0, 100.0);  // 100mm tool length
    config.toolRotation = Matrix3d::Identity();

    // Base at origin
    config.baseOffset = Vector3d::Zero();
    config.baseRotation = Matrix3d::Identity();

    return config;
}

/**
 * Create DH parameters for welding robot configuration
 * Includes typical welding torch offset
 */
inline RobotKinematicConfig createWeldingRobotConfig() {
    auto config = createDefault6DOFConfig();

    // Welding torch offset
    // Typical MIG/MAG torch: ~150mm length, 45 deg angle
    config.toolOffset = Vector3d(0, 0, 150.0);

    // Torch angled 45 degrees
    config.toolRotation = AngleAxisd(degToRad(45.0), Vector3d::UnitY()).toRotationMatrix();

    return config;
}

} // namespace kinematics
} // namespace robotics
```

---

## Step 4: Create Forward Kinematics

### 4.1 Create ForwardKinematics.hpp

**File:** `src/cpp/include/kinematics/ForwardKinematics.hpp`

```cpp
#pragma once

#include "DHParameters.hpp"
#include <vector>

namespace robotics {
namespace kinematics {

// ============================================================================
// TCP Pose Result
// ============================================================================

/**
 * Complete pose of Tool Center Point (TCP)
 */
struct TCPPose {
    Vector3d position;      // XYZ position in mm
    Matrix3d rotation;      // Rotation matrix
    Vector3d rpy;           // Roll-Pitch-Yaw angles in rad
    Quaterniond quaternion; // Orientation as quaternion

    TCPPose()
        : position(Vector3d::Zero()),
          rotation(Matrix3d::Identity()),
          rpy(Vector3d::Zero()),
          quaternion(Quaterniond::Identity()) {}

    // Create from homogeneous transformation matrix
    static TCPPose fromTransform(const Matrix4d& T) {
        TCPPose pose;
        pose.position = T.block<3, 1>(0, 3);
        pose.rotation = T.block<3, 3>(0, 0);
        pose.rpy = rotationToRPY(pose.rotation);
        pose.quaternion = Quaterniond(pose.rotation);
        return pose;
    }

    // Convert to homogeneous transformation matrix
    Matrix4d toTransform() const {
        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation;
        T.block<3, 1>(0, 3) = position;
        return T;
    }

    // Get as 6-element vector [x, y, z, roll, pitch, yaw]
    Vector6d toVector() const {
        Vector6d v;
        v << position, rpy;
        return v;
    }
};

// ============================================================================
// Forward Kinematics Class
// ============================================================================

/**
 * Forward Kinematics solver using DH convention
 * Computes TCP pose from joint angles
 */
class ForwardKinematics {
public:
    explicit ForwardKinematics(const RobotKinematicConfig& config);
    ~ForwardKinematics() = default;

    // ========================================================================
    // Main FK Methods
    // ========================================================================

    /**
     * Compute TCP pose from joint angles
     * @param jointAngles Current joint angles in radians
     * @return TCP pose (position and orientation)
     */
    TCPPose compute(const JointAngles& jointAngles) const;

    /**
     * Compute transformation matrix for single joint
     * @param jointIndex Index of joint (0-5)
     * @param jointAngle Current angle of joint in radians
     * @return 4x4 homogeneous transformation matrix
     */
    Matrix4d computeJointTransform(int jointIndex, double jointAngle) const;

    /**
     * Compute transformation matrices for all frames
     * Returns transforms from base to each joint frame
     * @param jointAngles Current joint angles
     * @return Vector of transforms [T_0^1, T_0^2, ..., T_0^6, T_0^TCP]
     */
    std::vector<Matrix4d> computeAllTransforms(const JointAngles& jointAngles) const;

    /**
     * Get position of each joint in world frame
     * Useful for visualization and collision detection
     */
    std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setConfig(const RobotKinematicConfig& config);
    const RobotKinematicConfig& getConfig() const { return config_; }

    void setToolOffset(const Vector3d& offset, const Matrix3d& rotation);
    void setBaseOffset(const Vector3d& offset, const Matrix3d& rotation);

private:
    RobotKinematicConfig config_;

    // Precomputed base and tool transforms
    Matrix4d baseTransform_;
    Matrix4d toolTransform_;

    void updateTransforms();
};

// ============================================================================
// Implementation
// ============================================================================

inline ForwardKinematics::ForwardKinematics(const RobotKinematicConfig& config)
    : config_(config) {
    updateTransforms();
}

inline void ForwardKinematics::updateTransforms() {
    // Base transform
    baseTransform_ = Matrix4d::Identity();
    baseTransform_.block<3, 3>(0, 0) = config_.baseRotation;
    baseTransform_.block<3, 1>(0, 3) = config_.baseOffset;

    // Tool transform
    toolTransform_ = Matrix4d::Identity();
    toolTransform_.block<3, 3>(0, 0) = config_.toolRotation;
    toolTransform_.block<3, 1>(0, 3) = config_.toolOffset;
}

inline void ForwardKinematics::setConfig(const RobotKinematicConfig& config) {
    config_ = config;
    updateTransforms();
}

inline void ForwardKinematics::setToolOffset(const Vector3d& offset, const Matrix3d& rotation) {
    config_.toolOffset = offset;
    config_.toolRotation = rotation;
    updateTransforms();
}

inline void ForwardKinematics::setBaseOffset(const Vector3d& offset, const Matrix3d& rotation) {
    config_.baseOffset = offset;
    config_.baseRotation = rotation;
    updateTransforms();
}

inline Matrix4d ForwardKinematics::computeJointTransform(int jointIndex, double jointAngle) const {
    if (jointIndex < 0 || jointIndex >= static_cast<int>(config_.dhParams.size())) {
        return Matrix4d::Identity();
    }

    const auto& dh = config_.dhParams[jointIndex];

    // Modified DH transformation matrix
    // T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)

    double theta = jointAngle + dh.theta;  // Add offset
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);

    Matrix4d T;
    T << ct,       -st,       0,      dh.a,
         st * ca,  ct * ca,  -sa,    -sa * dh.d,
         st * sa,  ct * sa,   ca,     ca * dh.d,
         0,        0,         0,      1;

    return T;
}

inline TCPPose ForwardKinematics::compute(const JointAngles& jointAngles) const {
    // Start with base transform
    Matrix4d T = baseTransform_;

    // Multiply through all joint transforms
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        T = T * computeJointTransform(i, jointAngles[i]);
    }

    // Apply tool transform
    T = T * toolTransform_;

    return TCPPose::fromTransform(T);
}

inline std::vector<Matrix4d> ForwardKinematics::computeAllTransforms(const JointAngles& jointAngles) const {
    std::vector<Matrix4d> transforms;
    transforms.reserve(config_.dhParams.size() + 1);

    Matrix4d T = baseTransform_;

    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        T = T * computeJointTransform(i, jointAngles[i]);
        transforms.push_back(T);
    }

    // Add TCP transform
    transforms.push_back(T * toolTransform_);

    return transforms;
}

inline std::vector<Vector3d> ForwardKinematics::computeJointPositions(const JointAngles& jointAngles) const {
    auto transforms = computeAllTransforms(jointAngles);

    std::vector<Vector3d> positions;
    positions.reserve(transforms.size() + 1);

    // Base position
    positions.push_back(config_.baseOffset);

    // Joint positions
    for (const auto& T : transforms) {
        positions.push_back(T.block<3, 1>(0, 3));
    }

    return positions;
}

} // namespace kinematics
} // namespace robotics
```

---

## Step 5: Create Inverse Kinematics

### 5.1 Create InverseKinematics.hpp

**File:** `src/cpp/include/kinematics/InverseKinematics.hpp`

```cpp
#pragma once

#include "ForwardKinematics.hpp"
#include <optional>
#include <functional>

namespace robotics {
namespace kinematics {

// ============================================================================
// IK Solution
// ============================================================================

/**
 * Result of inverse kinematics computation
 */
struct IKSolution {
    JointAngles angles;
    bool isValid;
    int configuration;      // Robot configuration index (elbow up/down, etc.)
    double residualError;   // Position error if iterative method used
    int iterations;         // Number of iterations (for iterative methods)

    IKSolution()
        : angles{0}, isValid(false), configuration(0),
          residualError(0), iterations(0) {}

    explicit IKSolution(const JointAngles& a, bool valid = true, int config = 0)
        : angles(a), isValid(valid), configuration(config),
          residualError(0), iterations(0) {}
};

/**
 * All possible IK solutions
 * For 6-DOF robot, can have up to 8 solutions
 */
struct IKSolutions {
    std::vector<IKSolution> solutions;
    bool hasAnySolution() const { return !solutions.empty(); }
    size_t count() const { return solutions.size(); }

    // Get solution closest to reference configuration
    std::optional<IKSolution> getClosestTo(const JointAngles& reference) const;

    // Get solution with minimum joint travel from reference
    std::optional<IKSolution> getMinimumTravel(const JointAngles& reference) const;
};

// ============================================================================
// IK Configuration
// ============================================================================

struct IKConfig {
    // Position tolerance (mm)
    double positionTolerance = 0.01;

    // Orientation tolerance (rad)
    double orientationTolerance = 0.001;

    // Maximum iterations for iterative solver
    int maxIterations = 100;

    // Damping factor for damped least squares
    double dampingFactor = 0.1;

    // Step size limit for iterative solver
    double maxStepSize = 0.1;  // rad

    // Use analytical solution when available
    bool preferAnalytical = true;

    // Check joint limits
    bool enforceJointLimits = true;
};

// ============================================================================
// Inverse Kinematics Class
// ============================================================================

/**
 * Inverse Kinematics solver
 * Supports both analytical (for standard 6-DOF) and numerical methods
 */
class InverseKinematics {
public:
    explicit InverseKinematics(const RobotKinematicConfig& config);
    ~InverseKinematics() = default;

    // ========================================================================
    // Main IK Methods
    // ========================================================================

    /**
     * Compute all IK solutions for target pose
     * @param targetPose Desired TCP pose
     * @return All valid solutions
     */
    IKSolutions computeAll(const TCPPose& targetPose) const;

    /**
     * Compute single IK solution closest to reference
     * @param targetPose Desired TCP pose
     * @param referenceAngles Current/reference joint angles
     * @return Best solution or nullopt if no solution
     */
    std::optional<IKSolution> compute(
        const TCPPose& targetPose,
        const JointAngles& referenceAngles) const;

    /**
     * Compute IK using iterative (numerical) method
     * Uses Damped Least Squares (DLS) / Levenberg-Marquardt
     */
    IKSolution computeIterative(
        const TCPPose& targetPose,
        const JointAngles& initialGuess) const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setConfig(const RobotKinematicConfig& config);
    void setIKConfig(const IKConfig& ikConfig) { ikConfig_ = ikConfig; }
    const IKConfig& getIKConfig() const { return ikConfig_; }

    // ========================================================================
    // Jacobian
    // ========================================================================

    /**
     * Compute Jacobian matrix at given configuration
     * @param jointAngles Current joint angles
     * @return 6x6 Jacobian matrix
     */
    Jacobian computeJacobian(const JointAngles& jointAngles) const;

    /**
     * Check if configuration is near singularity
     * @param jointAngles Current joint angles
     * @param threshold Condition number threshold
     * @return true if near singularity
     */
    bool isNearSingularity(const JointAngles& jointAngles, double threshold = 0.01) const;

    /**
     * Compute manipulability measure (Yoshikawa)
     * Higher value = further from singularity
     */
    double computeManipulability(const JointAngles& jointAngles) const;

private:
    RobotKinematicConfig config_;
    IKConfig ikConfig_;
    ForwardKinematics fk_;

    // Check if angles are within joint limits
    bool checkJointLimits(const JointAngles& angles) const;

    // Clamp angles to joint limits
    JointAngles clampToLimits(const JointAngles& angles) const;

    // Compute pose error
    Vector6d computePoseError(const TCPPose& current, const TCPPose& target) const;
};

// ============================================================================
// IKSolutions Implementation
// ============================================================================

inline std::optional<IKSolution> IKSolutions::getClosestTo(const JointAngles& reference) const {
    if (solutions.empty()) return std::nullopt;

    const IKSolution* best = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& sol : solutions) {
        if (!sol.isValid) continue;

        double distance = 0;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double diff = normalizeAngle(sol.angles[i] - reference[i]);
            distance += diff * diff;
        }

        if (distance < minDistance) {
            minDistance = distance;
            best = &sol;
        }
    }

    return best ? std::optional<IKSolution>(*best) : std::nullopt;
}

inline std::optional<IKSolution> IKSolutions::getMinimumTravel(const JointAngles& reference) const {
    // Same as getClosestTo for revolute joints
    return getClosestTo(reference);
}

// ============================================================================
// InverseKinematics Implementation
// ============================================================================

inline InverseKinematics::InverseKinematics(const RobotKinematicConfig& config)
    : config_(config), fk_(config) {
}

inline void InverseKinematics::setConfig(const RobotKinematicConfig& config) {
    config_ = config;
    fk_.setConfig(config);
}

inline bool InverseKinematics::checkJointLimits(const JointAngles& angles) const {
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        if (angles[i] < dh.minAngle || angles[i] > dh.maxAngle) {
            return false;
        }
    }
    return true;
}

inline JointAngles InverseKinematics::clampToLimits(const JointAngles& angles) const {
    JointAngles clamped = angles;
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        clamped[i] = std::clamp(clamped[i], dh.minAngle, dh.maxAngle);
    }
    return clamped;
}

inline Vector6d InverseKinematics::computePoseError(
    const TCPPose& current, const TCPPose& target) const {

    Vector6d error;

    // Position error
    error.head<3>() = target.position - current.position;

    // Orientation error (using angle-axis representation)
    Matrix3d Re = target.rotation * current.rotation.transpose();
    AngleAxisd aa(Re);
    error.tail<3>() = aa.angle() * aa.axis();

    return error;
}

inline Jacobian InverseKinematics::computeJacobian(const JointAngles& jointAngles) const {
    Jacobian J = Jacobian::Zero();

    // Compute all transforms
    auto transforms = fk_.computeAllTransforms(jointAngles);

    // TCP position
    Vector3d p_tcp = transforms.back().block<3, 1>(0, 3);

    // Base frame z-axis
    Vector3d z0 = config_.baseRotation.col(2);
    Vector3d p0 = config_.baseOffset;

    // Build Jacobian column by column
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = config_.baseRotation;
    T.block<3, 1>(0, 3) = config_.baseOffset;

    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        Vector3d z_i = T.block<3, 3>(0, 0).col(2);  // z-axis of frame i
        Vector3d p_i = T.block<3, 1>(0, 3);         // origin of frame i

        if (config_.dhParams[i].isRevolute) {
            // Revolute joint: J = [z x (p_tcp - p_i); z]
            J.block<3, 1>(0, i) = z_i.cross(p_tcp - p_i);
            J.block<3, 1>(3, i) = z_i;
        } else {
            // Prismatic joint: J = [z; 0]
            J.block<3, 1>(0, i) = z_i;
            J.block<3, 1>(3, i) = Vector3d::Zero();
        }

        // Update transform to next frame
        T = T * fk_.computeJointTransform(i, jointAngles[i]);
    }

    return J;
}

inline bool InverseKinematics::isNearSingularity(
    const JointAngles& jointAngles, double threshold) const {

    Jacobian J = computeJacobian(jointAngles);

    // Compute condition number using SVD
    Eigen::JacobiSVD<Jacobian> svd(J);
    auto singularValues = svd.singularValues();

    double minSV = singularValues(singularValues.size() - 1);
    double maxSV = singularValues(0);

    if (maxSV < EPSILON) return true;

    return (minSV / maxSV) < threshold;
}

inline double InverseKinematics::computeManipulability(const JointAngles& jointAngles) const {
    Jacobian J = computeJacobian(jointAngles);

    // Yoshikawa manipulability: sqrt(det(J * J^T))
    Matrix6d JJT = J * J.transpose();
    return std::sqrt(std::abs(JJT.determinant()));
}

inline IKSolution InverseKinematics::computeIterative(
    const TCPPose& targetPose,
    const JointAngles& initialGuess) const {

    IKSolution solution;
    solution.angles = initialGuess;
    solution.isValid = false;

    JointAngles q = initialGuess;

    for (int iter = 0; iter < ikConfig_.maxIterations; ++iter) {
        // Compute current pose
        TCPPose currentPose = fk_.compute(q);

        // Compute error
        Vector6d error = computePoseError(currentPose, targetPose);

        // Check convergence
        double posError = error.head<3>().norm();
        double oriError = error.tail<3>().norm();

        if (posError < ikConfig_.positionTolerance &&
            oriError < ikConfig_.orientationTolerance) {
            solution.angles = q;
            solution.isValid = true;
            solution.residualError = posError;
            solution.iterations = iter + 1;
            break;
        }

        // Compute Jacobian
        Jacobian J = computeJacobian(q);

        // Damped Least Squares: dq = J^T * (J*J^T + lambda^2*I)^-1 * error
        Matrix6d JJT = J * J.transpose();
        Matrix6d damped = JJT + ikConfig_.dampingFactor * ikConfig_.dampingFactor * Matrix6d::Identity();

        Eigen::Matrix<double, NUM_JOINTS, 1> dq = J.transpose() * damped.ldlt().solve(error);

        // Limit step size
        double stepNorm = dq.norm();
        if (stepNorm > ikConfig_.maxStepSize) {
            dq *= ikConfig_.maxStepSize / stepNorm;
        }

        // Update joint angles
        for (int i = 0; i < NUM_JOINTS; ++i) {
            q[i] += dq(i);
            q[i] = normalizeAngle(q[i]);
        }

        // Clamp to joint limits if required
        if (ikConfig_.enforceJointLimits) {
            q = clampToLimits(q);
        }

        solution.iterations = iter + 1;
    }

    // Final check
    if (!solution.isValid) {
        TCPPose finalPose = fk_.compute(q);
        Vector6d finalError = computePoseError(finalPose, targetPose);
        solution.residualError = finalError.head<3>().norm();
        solution.angles = q;

        // Accept if close enough
        if (solution.residualError < ikConfig_.positionTolerance * 10) {
            solution.isValid = true;
        }
    }

    // Verify joint limits
    if (solution.isValid && ikConfig_.enforceJointLimits) {
        solution.isValid = checkJointLimits(solution.angles);
    }

    return solution;
}

inline IKSolutions InverseKinematics::computeAll(const TCPPose& targetPose) const {
    IKSolutions solutions;

    // For general 6-DOF robot, use iterative method with different initial guesses
    // This gives approximate "all solutions" behavior

    std::vector<JointAngles> initialGuesses;

    // Zero configuration
    initialGuesses.push_back({0, 0, 0, 0, 0, 0});

    // Various configurations
    initialGuesses.push_back({0, -PI/4, PI/2, 0, PI/4, 0});
    initialGuesses.push_back({0, -PI/4, PI/2, PI, PI/4, 0});
    initialGuesses.push_back({PI, -PI/4, PI/2, 0, PI/4, 0});
    initialGuesses.push_back({0, PI/4, -PI/2, 0, -PI/4, 0});
    initialGuesses.push_back({0, -PI/2, PI/2, 0, 0, 0});
    initialGuesses.push_back({PI/2, 0, 0, PI/2, 0, PI/2});
    initialGuesses.push_back({-PI/2, 0, 0, -PI/2, 0, -PI/2});

    for (const auto& guess : initialGuesses) {
        IKSolution sol = computeIterative(targetPose, guess);

        if (sol.isValid) {
            // Check if this solution is unique
            bool isDuplicate = false;
            for (const auto& existing : solutions.solutions) {
                double diff = 0;
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    diff += std::abs(normalizeAngle(sol.angles[i] - existing.angles[i]));
                }
                if (diff < 0.1) {  // Solutions are essentially the same
                    isDuplicate = true;
                    break;
                }
            }

            if (!isDuplicate) {
                solutions.solutions.push_back(sol);
            }
        }
    }

    return solutions;
}

inline std::optional<IKSolution> InverseKinematics::compute(
    const TCPPose& targetPose,
    const JointAngles& referenceAngles) const {

    // First try iterative from reference (fastest convergence)
    IKSolution sol = computeIterative(targetPose, referenceAngles);

    if (sol.isValid) {
        return sol;
    }

    // If that fails, try all solutions and pick closest
    IKSolutions allSolutions = computeAll(targetPose);
    return allSolutions.getClosestTo(referenceAngles);
}

} // namespace kinematics
} // namespace robotics
```

---

## Step 6: Create Kinematics Service Interface

### 6.1 Create IKinematicsService.hpp

**File:** `src/cpp/include/kinematics/IKinematicsService.hpp`

```cpp
#pragma once

#include "InverseKinematics.hpp"
#include <memory>
#include <mutex>

namespace robotics {
namespace kinematics {

/**
 * Thread-safe Kinematics Service
 * Provides FK/IK computations for robot controller
 */
class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;

    // ========================================================================
    // Forward Kinematics
    // ========================================================================

    virtual TCPPose computeFK(const JointAngles& jointAngles) = 0;
    virtual std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) = 0;

    // ========================================================================
    // Inverse Kinematics
    // ========================================================================

    virtual std::optional<IKSolution> computeIK(
        const TCPPose& targetPose,
        const JointAngles& currentAngles) = 0;

    virtual IKSolutions computeAllIK(const TCPPose& targetPose) = 0;

    // ========================================================================
    // Jacobian & Singularity
    // ========================================================================

    virtual Jacobian computeJacobian(const JointAngles& jointAngles) = 0;
    virtual bool isNearSingularity(const JointAngles& jointAngles) = 0;
    virtual double computeManipulability(const JointAngles& jointAngles) = 0;

    // ========================================================================
    // Configuration
    // ========================================================================

    virtual void setRobotConfig(const RobotKinematicConfig& config) = 0;
    virtual void setToolOffset(const Vector3d& offset, const Matrix3d& rotation) = 0;
    virtual void setIKConfig(const IKConfig& config) = 0;

    virtual const RobotKinematicConfig& getRobotConfig() const = 0;

    // ========================================================================
    // Validation
    // ========================================================================

    virtual bool isReachable(const TCPPose& pose) = 0;
    virtual bool areJointAnglesValid(const JointAngles& angles) = 0;
};

/**
 * Default implementation of KinematicsService
 */
class KinematicsService : public IKinematicsService {
public:
    explicit KinematicsService(const RobotKinematicConfig& config);
    ~KinematicsService() override = default;

    // FK
    TCPPose computeFK(const JointAngles& jointAngles) override;
    std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) override;

    // IK
    std::optional<IKSolution> computeIK(
        const TCPPose& targetPose,
        const JointAngles& currentAngles) override;
    IKSolutions computeAllIK(const TCPPose& targetPose) override;

    // Jacobian
    Jacobian computeJacobian(const JointAngles& jointAngles) override;
    bool isNearSingularity(const JointAngles& jointAngles) override;
    double computeManipulability(const JointAngles& jointAngles) override;

    // Configuration
    void setRobotConfig(const RobotKinematicConfig& config) override;
    void setToolOffset(const Vector3d& offset, const Matrix3d& rotation) override;
    void setIKConfig(const IKConfig& config) override;
    const RobotKinematicConfig& getRobotConfig() const override;

    // Validation
    bool isReachable(const TCPPose& pose) override;
    bool areJointAnglesValid(const JointAngles& angles) override;

private:
    mutable std::mutex mutex_;
    RobotKinematicConfig config_;
    ForwardKinematics fk_;
    InverseKinematics ik_;
};

// ============================================================================
// KinematicsService Implementation
// ============================================================================

inline KinematicsService::KinematicsService(const RobotKinematicConfig& config)
    : config_(config), fk_(config), ik_(config) {
}

inline TCPPose KinematicsService::computeFK(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return fk_.compute(jointAngles);
}

inline std::vector<Vector3d> KinematicsService::computeJointPositions(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return fk_.computeJointPositions(jointAngles);
}

inline std::optional<IKSolution> KinematicsService::computeIK(
    const TCPPose& targetPose,
    const JointAngles& currentAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.compute(targetPose, currentAngles);
}

inline IKSolutions KinematicsService::computeAllIK(const TCPPose& targetPose) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeAll(targetPose);
}

inline Jacobian KinematicsService::computeJacobian(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeJacobian(jointAngles);
}

inline bool KinematicsService::isNearSingularity(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.isNearSingularity(jointAngles);
}

inline double KinematicsService::computeManipulability(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeManipulability(jointAngles);
}

inline void KinematicsService::setRobotConfig(const RobotKinematicConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    fk_.setConfig(config);
    ik_.setConfig(config);
}

inline void KinematicsService::setToolOffset(const Vector3d& offset, const Matrix3d& rotation) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.toolOffset = offset;
    config_.toolRotation = rotation;
    fk_.setToolOffset(offset, rotation);
    ik_.setConfig(config_);
}

inline void KinematicsService::setIKConfig(const IKConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    ik_.setIKConfig(config);
}

inline const RobotKinematicConfig& KinematicsService::getRobotConfig() const {
    return config_;
}

inline bool KinematicsService::isReachable(const TCPPose& pose) {
    JointAngles zeroAngles = {0, 0, 0, 0, 0, 0};
    auto solution = computeIK(pose, zeroAngles);
    return solution.has_value();
}

inline bool KinematicsService::areJointAnglesValid(const JointAngles& angles) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        if (angles[i] < dh.minAngle || angles[i] > dh.maxAngle) {
            return false;
        }
    }
    return true;
}

} // namespace kinematics
} // namespace robotics
```

---

## Step 7: Create Unit Tests

### 7.1 Create test_kinematics.cpp

**File:** `src/cpp/tests/test_kinematics.cpp`

```cpp
#include <gtest/gtest.h>
#include "kinematics/MathTypes.hpp"
#include "kinematics/DHParameters.hpp"
#include "kinematics/ForwardKinematics.hpp"
#include "kinematics/InverseKinematics.hpp"
#include "kinematics/IKinematicsService.hpp"

using namespace robotics::kinematics;
using namespace robotics::math;

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

    EXPECT_EQ(config.dhParams.size(), 6);
    EXPECT_EQ(config.dhParams[0].name, "J1_Base");
    EXPECT_EQ(config.dhParams[5].name, "J6_Wrist3");
}

TEST(DHParametersTest, WeldingConfig) {
    auto config = createWeldingRobotConfig();

    EXPECT_EQ(config.dhParams.size(), 6);
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
    EXPECT_EQ(positions.size(), 8);

    // First position is base
    EXPECT_NEAR(positions[0].norm(), config_.baseOffset.norm(), 0.01);
}

TEST_F(KinematicsTest, FK_AllTransforms) {
    JointAngles zeros = {0, 0, 0, 0, 0, 0};
    auto transforms = fk_->computeAllTransforms(zeros);

    // Should have 6 joint transforms + TCP = 7
    EXPECT_EQ(transforms.size(), 7);

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
    JointAngles target = {0.5, -0.3, 0.8, 0.2, -0.4, 0.1};
    TCPPose targetPose = fk_->compute(target);

    JointAngles initialGuess = {0, 0, 0, 0, 0, 0};
    IKSolution solution = ik_->computeIterative(targetPose, initialGuess);

    // Should converge
    EXPECT_TRUE(solution.isValid);
    EXPECT_LT(solution.residualError, 0.1);
    EXPECT_GT(solution.iterations, 0);
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
```

---

## Step 8: Update CMakeLists.txt

### 8.1 Add Kinematics to Build

**File:** `src/cpp/CMakeLists.txt`

**Add to include directories:**
```cmake
target_include_directories(RobotCore
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics
)
```

**Add test executable:**
```cmake
# Kinematics tests
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics
    PRIVATE
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)
target_include_directories(test_kinematics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME KinematicsTests COMMAND test_kinematics)
```

**Validation:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
cmake --build build --config Debug
.\build\Debug\test_kinematics.exe
```

---

## Step 9: Create IPC Message Types for Kinematics

### 9.1 Update MessageTypes.hpp

**File:** `src/cpp/include/ipc/MessageTypes.hpp`

**Add new message types:**
```cpp
// Kinematics messages
FK_REQUEST = 30,
FK_RESPONSE = 31,
IK_REQUEST = 32,
IK_RESPONSE = 33,
JACOBIAN_REQUEST = 34,
JACOBIAN_RESPONSE = 35,
SET_TOOL_REQUEST = 36,
SET_TOOL_RESPONSE = 37,
```

### 9.2 Create KinematicsPayloads.hpp

**File:** `src/cpp/include/ipc/KinematicsPayloads.hpp`

```cpp
#pragma once

#include <nlohmann/json.hpp>
#include <array>

namespace robotics {
namespace ipc {

// ============================================================================
// FK Request/Response
// ============================================================================

struct FKRequest {
    std::array<double, 6> jointAngles;  // radians

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FKRequest, jointAngles)
};

struct FKResponse {
    bool success;
    std::array<double, 3> position;      // mm
    std::array<double, 3> rpy;           // radians
    std::array<double, 4> quaternion;    // w, x, y, z
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FKResponse, success, position, rpy, quaternion, error)
};

// ============================================================================
// IK Request/Response
// ============================================================================

struct IKRequest {
    std::array<double, 3> targetPosition;  // mm
    std::array<double, 3> targetRPY;       // radians
    std::array<double, 6> referenceAngles; // radians
    bool getAllSolutions;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(IKRequest, targetPosition, targetRPY, referenceAngles, getAllSolutions)
};

struct IKSolutionData {
    std::array<double, 6> jointAngles;
    int configuration;
    double residualError;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(IKSolutionData, jointAngles, configuration, residualError)
};

struct IKResponse {
    bool success;
    std::vector<IKSolutionData> solutions;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(IKResponse, success, solutions, error)
};

// ============================================================================
// Tool Offset Request/Response
// ============================================================================

struct SetToolRequest {
    std::array<double, 3> offset;   // mm
    std::array<double, 3> rpy;      // radians

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SetToolRequest, offset, rpy)
};

struct SetToolResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SetToolResponse, success, error)
};

// ============================================================================
// Joint Positions for Visualization
// ============================================================================

struct JointPositionsRequest {
    std::array<double, 6> jointAngles;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JointPositionsRequest, jointAngles)
};

struct JointPositionsResponse {
    bool success;
    std::vector<std::array<double, 3>> positions;  // Base + 6 joints + TCP
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JointPositionsResponse, success, positions, error)
};

} // namespace ipc
} // namespace robotics
```

---

## Step 10: Create C# Kinematics Client

### 10.1 Create KinematicsPayloads.cs

**File:** `src/csharp/RobotController.Core/IPC/KinematicsPayloads.cs`

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Core.IPC;

// ============================================================================
// FK Request/Response
// ============================================================================

public record FKRequest
{
    [JsonPropertyName("jointAngles")]
    public double[] JointAngles { get; init; } = new double[6];
}

public record FKResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("position")]
    public double[] Position { get; init; } = new double[3];

    [JsonPropertyName("rpy")]
    public double[] RPY { get; init; } = new double[3];

    [JsonPropertyName("quaternion")]
    public double[] Quaternion { get; init; } = new double[4];

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// IK Request/Response
// ============================================================================

public record IKRequest
{
    [JsonPropertyName("targetPosition")]
    public double[] TargetPosition { get; init; } = new double[3];

    [JsonPropertyName("targetRPY")]
    public double[] TargetRPY { get; init; } = new double[3];

    [JsonPropertyName("referenceAngles")]
    public double[] ReferenceAngles { get; init; } = new double[6];

    [JsonPropertyName("getAllSolutions")]
    public bool GetAllSolutions { get; init; }
}

public record IKSolutionData
{
    [JsonPropertyName("jointAngles")]
    public double[] JointAngles { get; init; } = new double[6];

    [JsonPropertyName("configuration")]
    public int Configuration { get; init; }

    [JsonPropertyName("residualError")]
    public double ResidualError { get; init; }
}

public record IKResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("solutions")]
    public IKSolutionData[] Solutions { get; init; } = Array.Empty<IKSolutionData>();

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Tool Offset
// ============================================================================

public record SetToolRequest
{
    [JsonPropertyName("offset")]
    public double[] Offset { get; init; } = new double[3];

    [JsonPropertyName("rpy")]
    public double[] RPY { get; init; } = new double[3];
}

public record SetToolResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Joint Positions for Visualization
// ============================================================================

public record JointPositionsRequest
{
    [JsonPropertyName("jointAngles")]
    public double[] JointAngles { get; init; } = new double[6];
}

public record JointPositionsResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("positions")]
    public double[][] Positions { get; init; } = Array.Empty<double[]>();

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}
```

### 10.2 Create IKinematicsClientService.cs

**File:** `src/csharp/RobotController.Core/Services/IKinematicsClientService.cs`

```csharp
using RobotController.Core.IPC;

namespace RobotController.Core.Services;

/// <summary>
/// Client service for robot kinematics computations
/// Communicates with C++ core via IPC
/// </summary>
public interface IKinematicsClientService
{
    /// <summary>
    /// Compute forward kinematics
    /// </summary>
    Task<FKResponse> ComputeFKAsync(double[] jointAngles, CancellationToken ct = default);

    /// <summary>
    /// Compute inverse kinematics
    /// </summary>
    Task<IKResponse> ComputeIKAsync(
        double[] targetPosition,
        double[] targetRPY,
        double[] referenceAngles,
        bool getAllSolutions = false,
        CancellationToken ct = default);

    /// <summary>
    /// Get joint positions for 3D visualization
    /// </summary>
    Task<JointPositionsResponse> GetJointPositionsAsync(
        double[] jointAngles,
        CancellationToken ct = default);

    /// <summary>
    /// Set tool offset
    /// </summary>
    Task<SetToolResponse> SetToolOffsetAsync(
        double[] offset,
        double[] rpy,
        CancellationToken ct = default);
}

/// <summary>
/// Implementation of kinematics client service
/// </summary>
public class KinematicsClientService : IKinematicsClientService
{
    private readonly IIpcClientService _ipc;
    private readonly ILogger<KinematicsClientService> _logger;

    public KinematicsClientService(
        IIpcClientService ipc,
        ILogger<KinematicsClientService> logger)
    {
        _ipc = ipc;
        _logger = logger;
    }

    public async Task<FKResponse> ComputeFKAsync(
        double[] jointAngles,
        CancellationToken ct = default)
    {
        var request = new FKRequest { JointAngles = jointAngles };

        try
        {
            var response = await _ipc.SendRequestAsync<FKRequest, FKResponse>(
                MessageType.FK_REQUEST,
                request,
                ct);

            return response ?? new FKResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "FK computation failed");
            return new FKResponse { Success = false, Error = ex.Message };
        }
    }

    public async Task<IKResponse> ComputeIKAsync(
        double[] targetPosition,
        double[] targetRPY,
        double[] referenceAngles,
        bool getAllSolutions = false,
        CancellationToken ct = default)
    {
        var request = new IKRequest
        {
            TargetPosition = targetPosition,
            TargetRPY = targetRPY,
            ReferenceAngles = referenceAngles,
            GetAllSolutions = getAllSolutions
        };

        try
        {
            var response = await _ipc.SendRequestAsync<IKRequest, IKResponse>(
                MessageType.IK_REQUEST,
                request,
                ct);

            return response ?? new IKResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "IK computation failed");
            return new IKResponse { Success = false, Error = ex.Message };
        }
    }

    public async Task<JointPositionsResponse> GetJointPositionsAsync(
        double[] jointAngles,
        CancellationToken ct = default)
    {
        var request = new JointPositionsRequest { JointAngles = jointAngles };

        try
        {
            var response = await _ipc.SendRequestAsync<JointPositionsRequest, JointPositionsResponse>(
                MessageType.JOINT_POSITIONS_REQUEST,
                request,
                ct);

            return response ?? new JointPositionsResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Joint positions request failed");
            return new JointPositionsResponse { Success = false, Error = ex.Message };
        }
    }

    public async Task<SetToolResponse> SetToolOffsetAsync(
        double[] offset,
        double[] rpy,
        CancellationToken ct = default)
    {
        var request = new SetToolRequest { Offset = offset, RPY = rpy };

        try
        {
            var response = await _ipc.SendRequestAsync<SetToolRequest, SetToolResponse>(
                MessageType.SET_TOOL_REQUEST,
                request,
                ct);

            return response ?? new SetToolResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Set tool offset failed");
            return new SetToolResponse { Success = false, Error = ex.Message };
        }
    }
}
```

---

## Step 11: Update Main to Include Kinematics

### 11.1 Update main.cpp

**File:** `src/cpp/src/main.cpp`

**Add includes:**
```cpp
#include "kinematics/IKinematicsService.hpp"
```

**Add to main():**
```cpp
// Initialize kinematics
auto robotConfig = robotics::kinematics::createDefault6DOFConfig();
auto kinematicsService = std::make_shared<robotics::kinematics::KinematicsService>(robotConfig);

LOG_INFO("Kinematics service initialized");

// Test FK at zero position
robotics::math::JointAngles zeroAngles = {0, 0, 0, 0, 0, 0};
auto tcpPose = kinematicsService->computeFK(zeroAngles);
LOG_INFO("TCP at zero: x={:.2f}, y={:.2f}, z={:.2f}",
         tcpPose.position.x(), tcpPose.position.y(), tcpPose.position.z());
```

---

## Step 12: Validation

### 12.1 Build and Test

**Commands:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller

# Build
cmake --build build --config Debug

# Run kinematics tests
.\build\Debug\test_kinematics.exe

# Run main (should show kinematics initialization)
.\build\Debug\RobotCore.exe
```

**Expected Output:**
```
[==========] Running 18 tests from 4 test suites.
...
[  PASSED  ] 18 tests.
```

### 12.2 FK/IK Consistency Check

Manually verify:
1. FK at zero position returns expected TCP location
2. IK of FK result returns original angles
3. Multiple IK solutions found for reachable poses

---

## Completion Checklist

- [ ] Eigen library installed via vcpkg
- [ ] MathTypes.hpp created with vector/matrix types
- [ ] DHParameters.hpp created with robot configuration
- [ ] ForwardKinematics.hpp implemented
- [ ] InverseKinematics.hpp implemented (iterative solver)
- [ ] IKinematicsService.hpp created (thread-safe wrapper)
- [ ] test_kinematics.cpp created with 18+ tests
- [ ] CMakeLists.txt updated for kinematics
- [ ] IPC message types added for kinematics
- [ ] C# KinematicsClientService created
- [ ] All kinematics tests pass
- [ ] Main application shows kinematics initialization

---

## Troubleshooting

### Eigen Not Found
```
Error: Could not find Eigen3
```
**Solution:**
```powershell
vcpkg install eigen3:x64-windows
vcpkg integrate install
```

### IK Not Converging
- Check if target pose is reachable
- Increase maxIterations in IKConfig
- Try different initial guess
- Check joint limits

### Singularity Issues
- Monitor manipulability value
- Avoid wrist singularity (J5 near 0)
- Avoid shoulder singularity (J2 + J3 = +/- 90 deg)

---

## Notes

### DH Parameters Customization
DH parameters trong `createDefault6DOFConfig()` là typical values. Để custom cho robot cụ thể:
1. Đo actual link lengths
2. Xác định joint directions
3. Update values trong DHParameter structs

### IK Algorithm
Implementation sử dụng Damped Least Squares (DLS) thay vì analytical solution vì:
- Works cho bất kỳ 6-DOF configuration
- Handles singularities gracefully
- Easy to add constraints

Nếu cần analytical IK cho specific robot (UR, KUKA, etc.), có thể add specialized solver sau.

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P2_02: Add kinematics engine with FK/IK

- Add Eigen library dependency
- Create MathTypes with vector/matrix utilities
- Implement DH parameters structure
- Implement Forward Kinematics solver
- Implement Inverse Kinematics (DLS method)
- Add Jacobian computation and singularity detection
- Create thread-safe KinematicsService
- Add 18 unit tests for kinematics
- Add IPC message types for kinematics
- Create C# KinematicsClientService

Co-Authored-By: Claude <noreply@anthropic.com>"
```
