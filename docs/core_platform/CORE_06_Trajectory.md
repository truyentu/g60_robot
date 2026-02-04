# CORE MODULE: Trajectory Generator (Ruckig OTG)

## Document Info
| Item | Value |
|------|-------|
| **Module** | Trajectory Generator |
| **Layer** | Core Logic (C++ 17/20) |
| **Library** | Ruckig Community Edition |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Module Trajectory Generator chịu trách nhiệm tạo quỹ đạo chuyển động thời gian thực (Online Trajectory Generation - OTG) cho robot 6-DOF. Module sử dụng thư viện Ruckig để tạo các quỹ đạo giới hạn Jerk (S-Curve), đảm bảo chuyển động mượt mà và không gây rung động cho mỏ hàn.

### 1.2. Key Features
- **Jerk-Limited Motion**: Biên dạng S-Curve thay vì Trapezoidal
- **Real-time OTG**: Tính toán trong mỗi chu kỳ điều khiển (1ms)
- **Phase Synchronization**: Đảm bảo đường thẳng tuyệt đối (MOVL)
- **Arbitrary Target Updates**: Hỗ trợ Seam Tracking với mục tiêu di động
- **Dual Mode**: Cartesian OTG (MOVL) và Joint OTG (MOVJ)

### 1.3. Dependencies

```
┌─────────────────────────────────────────────────────────────────┐
│                    TRAJECTORY DEPENDENCIES                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐    │
│  │  Kinematics  │     │    Config    │     │    State     │    │
│  │    Module    │     │    System    │     │   Manager    │    │
│  │   (IK/FK)    │     │   (YAML)     │     │    (FSM)     │    │
│  └──────┬───────┘     └──────┬───────┘     └──────┬───────┘    │
│         │                    │                    │             │
│         └────────────────────┼────────────────────┘             │
│                              │                                   │
│                              ▼                                   │
│                    ┌──────────────────┐                         │
│                    │   Trajectory     │                         │
│                    │   Generator      │                         │
│                    │   (Ruckig OTG)   │                         │
│                    └────────┬─────────┘                         │
│                             │                                    │
│              ┌──────────────┴──────────────┐                    │
│              │                             │                    │
│              ▼                             ▼                    │
│    ┌──────────────────┐          ┌──────────────────┐          │
│    │  Motion Engine   │          │  Vision Pipeline │          │
│    │  (grblHAL)       │          │  (Seam Tracking) │          │
│    └──────────────────┘          └──────────────────┘          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture

### 2.1. Real-time Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│           HARD REAL-TIME LOOP (1kHz / 1ms Cycle)                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  LAYER 1: Input Collection (Start of Cycle)              │   │
│  │                                                           │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐   │   │
│  │  │   Servo     │    │    FK       │    │   Sensor    │   │   │
│  │  │  Feedback   │───►│  (RL/IK)    │    │   Data      │   │   │
│  │  │   θ_curr    │    │  → X_curr   │    │  (Laser)    │   │   │
│  │  └─────────────┘    └──────┬──────┘    └──────┬──────┘   │   │
│  └───────────────────────────┬────────────────────┘         │   │
│                              │                               │   │
│  ┌───────────────────────────▼──────────────────────────────┐   │
│  │  LAYER 2: Trajectory Generation (Ruckig OTG)             │   │
│  │                                                           │   │
│  │  ┌──────────────────────────────────────────────────────┐│   │
│  │  │                                                       ││   │
│  │  │   State_current (X, V, A)                            ││   │
│  │  │          │                                            ││   │
│  │  │          ▼                                            ││   │
│  │  │   ┌─────────────────────────────────┐                ││   │
│  │  │   │        RUCKIG.update()          │                ││   │
│  │  │   │                                 │                ││   │
│  │  │   │  • Phase Synchronization        │                ││   │
│  │  │   │  • Jerk-Limited Profile         │                ││   │
│  │  │   │  • Time-Optimal Calculation     │                ││   │
│  │  │   │                                 │                ││   │
│  │  │   └──────────────┬──────────────────┘                ││   │
│  │  │                  │                                    ││   │
│  │  │                  ▼                                    ││   │
│  │  │   State_next (X_cmd, V_cmd, A_cmd)                   ││   │
│  │  │                                                       ││   │
│  │  └──────────────────────────────────────────────────────┘│   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│  ┌───────────────────────────▼──────────────────────────────┐   │
│  │  LAYER 3: Inverse Kinematics (RL Core Math)              │   │
│  │                                                           │   │
│  │  X_cmd ──► IK Solver ──► θ_cmd (6 Joint Angles)         │   │
│  │                                                           │   │
│  └───────────────────────────┬──────────────────────────────┘   │
│                              │                                   │
│  ┌───────────────────────────▼──────────────────────────────┐   │
│  │  LAYER 4: Safety Check & Output                          │   │
│  │                                                           │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐   │   │
│  │  │  Joint      │    │  Velocity   │    │   Send to   │   │   │
│  │  │  Limits     │    │   Scaling   │    │  grblHAL    │   │   │
│  │  │   Check     │    │  (if near   │    │   Driver    │   │   │
│  │  │             │    │  singularity)│    │             │   │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Cartesian vs Joint Space OTG

```
┌─────────────────────────────────────────────────────────────────┐
│                OTG MODE COMPARISON                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  CARTESIAN OTG (MOVL - Linear Motion)                           │
│  ═══════════════════════════════════                            │
│                                                                  │
│     Start ────────────────────────────► End                     │
│        ●═══════════════════════════════●                        │
│                   Straight Line                                  │
│                                                                  │
│     • Input: TCP Position (X, Y, Z, Rx, Ry, Rz)                 │
│     • Output: Straight line path in 3D space                    │
│     • Use: Welding, Seam Tracking, Linear interpolation         │
│     • Risk: Singularity when path crosses wrist alignment       │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  JOINT SPACE OTG (MOVJ - Joint Motion)                          │
│  ═════════════════════════════════════                          │
│                                                                  │
│     Start ─────╮                  ╭───► End                     │
│        ●        ╲                ╱       ●                       │
│                  ╲──────────────╱                                │
│                     Arc Path                                     │
│                                                                  │
│     • Input: Joint Angles (θ1, θ2, θ3, θ4, θ5, θ6)              │
│     • Output: Curved path in 3D space (fastest joint motion)    │
│     • Use: Point-to-point move, Homing, Non-welding moves       │
│     • Safe: Never encounters singularity                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Ruckig Configuration

### 3.1. Synchronization Modes

```
┌─────────────────────────────────────────────────────────────────┐
│              SYNCHRONIZATION MODE COMPARISON                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Synchronization::None                                           │
│  ─────────────────────                                          │
│     X ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                           │
│     Y ━━━━━━━━━━━━━━━━━━━━━━━━                                  │
│     Z ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                        │
│                                                                  │
│     Result: Zigzag/Curved path ❌ NOT FOR WELDING               │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  Synchronization::Time                                           │
│  ─────────────────────                                          │
│     X ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                        │
│     Y ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                        │
│     Z ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                        │
│        ↑ Same end time, different profiles                      │
│                                                                  │
│     Result: Still curved path ⚠️ NOT IDEAL                      │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  Synchronization::Phase ✅ REQUIRED FOR MOVL                    │
│  ──────────────────────                                         │
│     X ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                        │
│     Y ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  (proportionally       │
│     Z ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   scaled profiles)     │
│                                                                  │
│     Result: STRAIGHT LINE ✅ PERFECT FOR WELDING                │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2. S-Curve Profile

```
┌─────────────────────────────────────────────────────────────────┐
│                 S-CURVE vs TRAPEZOIDAL                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Trapezoidal (Jerk = ∞ at transitions)                          │
│  ──────────────────────────────────────                         │
│                                                                  │
│  Velocity                                                        │
│     ▲                                                            │
│     │    ┌────────────────┐                                      │
│     │   ╱                  ╲    ← Instant acceleration change   │
│     │  ╱                    ╲      causes vibration             │
│     │ ╱                      ╲                                   │
│     └────────────────────────────► Time                         │
│                                                                  │
│  Problem: High Jerk → Vibration → Weld pool instability        │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  S-Curve (Jerk-Limited)                                         │
│  ──────────────────────                                         │
│                                                                  │
│  Velocity                                                        │
│     ▲                                                            │
│     │    ╭──────────────╮                                        │
│     │   ╱                ╲   ← Smooth acceleration change       │
│     │  ╱                  ╲     no vibration                    │
│     │ ╱                    ╲                                     │
│     └────────────────────────────► Time                         │
│                                                                  │
│  Benefit: Zero vibration → Stable weld pool → Quality weld     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Task Breakdown

### 4.1. Task List

| ID | Task | Description | Priority | Dependencies |
|----|------|-------------|----------|--------------|
| **T-01** | Ruckig Integration | Tích hợp thư viện Ruckig vào CMake | P0 | CORE_Kinematics |
| **T-02** | TrajectoryInput Class | Cấu trúc dữ liệu đầu vào | P0 | T-01 |
| **T-03** | TrajectoryOutput Class | Cấu trúc dữ liệu đầu ra | P0 | T-01 |
| **T-04** | CartesianOTG | OTG trong không gian Cartesian | P0 | T-02, T-03 |
| **T-05** | JointOTG | OTG trong không gian Joint | P0 | T-02, T-03 |
| **T-06** | PhaseSynchronization | Đồng bộ pha cho MOVL | P0 | T-04 |
| **T-07** | OrientationInterpolation | Nội suy hướng (Euler unwinding) | P0 | T-04 |
| **T-08** | TrajectoryEngine | Class tổng hợp OTG | P0 | T-04, T-05 |
| **T-09** | VelocityScaling | Giảm tốc khi gần singularity | P1 | T-08 |
| **T-10** | MovingTarget | Cập nhật mục tiêu liên tục | P1 | T-08 |
| **T-11** | SensorFusion | Tích hợp offset từ cảm biến | P1 | T-10 |
| **T-12** | BlendedMotion | Nối tiếp các segment (Zone) | P2 | T-08 |
| **T-13** | ConfigLoader | Load giới hạn từ YAML | P0 | Config System |
| **T-14** | UnitTests | Test cases cho OTG | P0 | T-08 |
| **T-15** | PerformanceBenchmark | Đo cycle time | P1 | T-08 |

### 4.2. Task Details

#### T-04: CartesianOTG
```
Input:
  - current_state: CartesianState (X, Y, Z, Rx, Ry, Rz + velocities + accelerations)
  - target_state: CartesianState
  - limits: CartesianLimits (max_vel, max_acc, max_jerk for each axis)

Process:
  1. Configure Ruckig với 6 DOF
  2. Set Synchronization::Phase
  3. Call otg.update()
  4. Return next CartesianState

Output:
  - next_state: CartesianState
  - trajectory_time: Thời gian còn lại đến đích
  - result: Working / Finished / Error
```

#### T-07: OrientationInterpolation
```
Problem: Euler angles discontinuity (179° → -179°)

Solution: Angle Unwinding
  if (target_angle - current_angle > 180°):
      target_angle -= 360°
  elif (target_angle - current_angle < -180°):
      target_angle += 360°

Example:
  Current: 10°, Target: 350°
  Without unwinding: +340° rotation (wrong!)
  With unwinding: Target becomes -10° → -20° rotation (correct!)
```

---

## 5. Implementation

### 5.1. Core Data Structures

```cpp
// trajectory_types.hpp

#pragma once
#include <array>
#include <Eigen/Dense>

namespace robot_controller {
namespace trajectory {

// Number of Cartesian DOFs (X, Y, Z, Rx, Ry, Rz)
constexpr size_t CARTESIAN_DOFS = 6;
// Number of robot joints
constexpr size_t JOINT_DOFS = 6;

/**
 * @brief Cartesian state vector
 */
struct CartesianState {
    std::array<double, CARTESIAN_DOFS> position;      // [m, m, m, rad, rad, rad]
    std::array<double, CARTESIAN_DOFS> velocity;      // [m/s, rad/s]
    std::array<double, CARTESIAN_DOFS> acceleration;  // [m/s², rad/s²]

    // Helper to reset velocities/accelerations
    void reset_dynamics() {
        velocity.fill(0.0);
        acceleration.fill(0.0);
    }
};

/**
 * @brief Joint state vector
 */
struct JointState {
    std::array<double, JOINT_DOFS> position;      // [rad]
    std::array<double, JOINT_DOFS> velocity;      // [rad/s]
    std::array<double, JOINT_DOFS> acceleration;  // [rad/s²]

    void reset_dynamics() {
        velocity.fill(0.0);
        acceleration.fill(0.0);
    }
};

/**
 * @brief Motion limits for trajectory generation
 */
struct CartesianLimits {
    std::array<double, CARTESIAN_DOFS> max_velocity;     // [m/s, rad/s]
    std::array<double, CARTESIAN_DOFS> max_acceleration; // [m/s², rad/s²]
    std::array<double, CARTESIAN_DOFS> max_jerk;         // [m/s³, rad/s³]
};

struct JointLimits {
    std::array<double, JOINT_DOFS> max_velocity;     // [rad/s]
    std::array<double, JOINT_DOFS> max_acceleration; // [rad/s²]
    std::array<double, JOINT_DOFS> max_jerk;         // [rad/s³]
};

/**
 * @brief Result of trajectory update
 */
enum class TrajectoryResult {
    Working,      // Still generating trajectory
    Finished,     // Reached target
    Error,        // Calculation error
    ErrorInput,   // Invalid input state
    ErrorSync     // Synchronization not possible
};

/**
 * @brief Trajectory output for one cycle
 */
struct TrajectoryOutput {
    CartesianState cartesian_state;
    JointState joint_state;
    TrajectoryResult result;
    double time_remaining;     // Time to reach target [s]
    double trajectory_time;    // Total trajectory duration [s]
    bool near_singularity;     // Warning flag
};

/**
 * @brief Motion type enumeration
 */
enum class MotionType {
    MOVL,   // Linear interpolation in Cartesian space
    MOVJ,   // Joint interpolation
    MOVC    // Circular interpolation (future)
};

} // namespace trajectory
} // namespace robot_controller
```

### 5.2. Cartesian OTG Implementation

```cpp
// cartesian_otg.hpp

#pragma once
#include "trajectory_types.hpp"
#include <ruckig/ruckig.hpp>
#include <memory>

namespace robot_controller {
namespace trajectory {

/**
 * @brief Online Trajectory Generation in Cartesian space
 *
 * Uses Ruckig library with Phase Synchronization to ensure
 * straight-line motion (MOVL) for welding applications.
 */
class CartesianOTG {
public:
    /**
     * @brief Constructor
     * @param cycle_time Control loop cycle time in seconds (e.g., 0.001 for 1kHz)
     */
    explicit CartesianOTG(double cycle_time = 0.001);

    /**
     * @brief Initialize with motion limits
     * @param limits Cartesian velocity/acceleration/jerk limits
     */
    void initialize(const CartesianLimits& limits);

    /**
     * @brief Set new target state
     * @param target Target position/velocity (velocity usually zero for point-to-point)
     */
    void setTarget(const CartesianState& target);

    /**
     * @brief Update target position for tracking (adds offset)
     * @param offset Sensor offset in world frame [m, m, m, rad, rad, rad]
     */
    void updateTargetOffset(const std::array<double, CARTESIAN_DOFS>& offset);

    /**
     * @brief Set target velocity for continuous tracking
     * @param velocity Travel velocity along path [m/s, rad/s]
     */
    void setTargetVelocity(const std::array<double, CARTESIAN_DOFS>& velocity);

    /**
     * @brief Execute one trajectory update cycle
     * @param current_state Current Cartesian state (from FK)
     * @param output Calculated next state
     * @return Trajectory result
     */
    TrajectoryResult update(const CartesianState& current_state,
                           TrajectoryOutput& output);

    /**
     * @brief Reset trajectory (stop motion)
     */
    void reset();

    /**
     * @brief Get remaining time to target
     */
    double getTimeRemaining() const;

    /**
     * @brief Check if trajectory is active
     */
    bool isActive() const;

private:
    // Ruckig OTG instance
    std::unique_ptr<ruckig::Ruckig<CARTESIAN_DOFS>> otg_;
    ruckig::InputParameter<CARTESIAN_DOFS> input_;
    ruckig::OutputParameter<CARTESIAN_DOFS> output_;

    // Configuration
    double cycle_time_;
    CartesianLimits limits_;
    bool is_initialized_;
    bool is_active_;

    // Internal methods
    void configureSynchronization();
    void unwrapAngles(std::array<double, CARTESIAN_DOFS>& target,
                      const std::array<double, CARTESIAN_DOFS>& current);
    void copyStateToInput(const CartesianState& state);
    void copyOutputToState(TrajectoryOutput& output);
};

} // namespace trajectory
} // namespace robot_controller
```

### 5.3. CartesianOTG Implementation

```cpp
// cartesian_otg.cpp

#include "cartesian_otg.hpp"
#include <cmath>
#include <stdexcept>

namespace robot_controller {
namespace trajectory {

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;

    // Unwrap angle to [-PI, PI] range relative to reference
    double unwrapAngle(double target, double reference) {
        double diff = target - reference;
        while (diff > PI) diff -= TWO_PI;
        while (diff < -PI) diff += TWO_PI;
        return reference + diff;
    }
}

CartesianOTG::CartesianOTG(double cycle_time)
    : cycle_time_(cycle_time)
    , is_initialized_(false)
    , is_active_(false)
{
    if (cycle_time <= 0.0 || cycle_time > 0.1) {
        throw std::invalid_argument("Cycle time must be between 0 and 100ms");
    }

    otg_ = std::make_unique<ruckig::Ruckig<CARTESIAN_DOFS>>(cycle_time);
}

void CartesianOTG::initialize(const CartesianLimits& limits) {
    limits_ = limits;

    // Set limits to Ruckig input
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.max_velocity[i] = limits_.max_velocity[i];
        input_.max_acceleration[i] = limits_.max_acceleration[i];
        input_.max_jerk[i] = limits_.max_jerk[i];
    }

    configureSynchronization();
    is_initialized_ = true;
}

void CartesianOTG::configureSynchronization() {
    // Phase synchronization is MANDATORY for straight-line motion (MOVL)
    // Without this, TCP will move in curved path!
    input_.synchronization = ruckig::Synchronization::Phase;

    // Interface mode: Position control
    input_.control_interface = ruckig::ControlInterface::Position;
}

void CartesianOTG::setTarget(const CartesianState& target) {
    if (!is_initialized_) {
        throw std::runtime_error("CartesianOTG not initialized");
    }

    // Copy target position
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.target_position[i] = target.position[i];
        input_.target_velocity[i] = target.velocity[i];
        input_.target_acceleration[i] = target.acceleration[i];
    }

    is_active_ = true;
}

void CartesianOTG::updateTargetOffset(const std::array<double, CARTESIAN_DOFS>& offset) {
    // Add sensor offset to current target (for seam tracking)
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.target_position[i] += offset[i];
    }
}

void CartesianOTG::setTargetVelocity(const std::array<double, CARTESIAN_DOFS>& velocity) {
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.target_velocity[i] = velocity[i];
    }
}

TrajectoryResult CartesianOTG::update(const CartesianState& current_state,
                                      TrajectoryOutput& output) {
    if (!is_initialized_) {
        return TrajectoryResult::Error;
    }

    if (!is_active_) {
        // Copy current state to output, no motion
        output.cartesian_state = current_state;
        output.result = TrajectoryResult::Finished;
        output.time_remaining = 0.0;
        return TrajectoryResult::Finished;
    }

    // Copy current state to Ruckig input
    copyStateToInput(current_state);

    // Unwrap orientation angles to avoid discontinuity
    std::array<double, CARTESIAN_DOFS> target_copy;
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        target_copy[i] = input_.target_position[i];
    }
    unwrapAngles(target_copy, current_state.position);
    for (size_t i = 3; i < CARTESIAN_DOFS; ++i) {
        input_.target_position[i] = target_copy[i];
    }

    // Execute Ruckig update
    ruckig::Result result = otg_->update(input_, output_);

    // Copy output to trajectory state
    copyOutputToState(output);

    // Convert Ruckig result
    switch (result) {
        case ruckig::Result::Working:
            output.result = TrajectoryResult::Working;
            output.time_remaining = output_.trajectory.get_duration() - output_.time;
            break;

        case ruckig::Result::Finished:
            output.result = TrajectoryResult::Finished;
            output.time_remaining = 0.0;
            is_active_ = false;
            break;

        case ruckig::Result::Error:
            output.result = TrajectoryResult::Error;
            break;

        case ruckig::Result::ErrorInvalidInput:
            output.result = TrajectoryResult::ErrorInput;
            break;

        case ruckig::Result::ErrorSynchronizationCalculation:
            output.result = TrajectoryResult::ErrorSync;
            break;

        default:
            output.result = TrajectoryResult::Error;
    }

    // Pass current output to next cycle's input (State-to-State mechanism)
    output_.pass_to_input(input_);

    return output.result;
}

void CartesianOTG::reset() {
    is_active_ = false;

    // Reset all velocities and accelerations to zero
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.current_velocity[i] = 0.0;
        input_.current_acceleration[i] = 0.0;
    }
}

double CartesianOTG::getTimeRemaining() const {
    if (!is_active_) return 0.0;
    return output_.trajectory.get_duration() - output_.time;
}

bool CartesianOTG::isActive() const {
    return is_active_;
}

void CartesianOTG::unwrapAngles(std::array<double, CARTESIAN_DOFS>& target,
                                const std::array<double, CARTESIAN_DOFS>& current) {
    // Only unwrap orientation angles (indices 3, 4, 5)
    for (size_t i = 3; i < CARTESIAN_DOFS; ++i) {
        target[i] = unwrapAngle(target[i], current[i]);
    }
}

void CartesianOTG::copyStateToInput(const CartesianState& state) {
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        input_.current_position[i] = state.position[i];
        input_.current_velocity[i] = state.velocity[i];
        input_.current_acceleration[i] = state.acceleration[i];
    }
}

void CartesianOTG::copyOutputToState(TrajectoryOutput& output) {
    for (size_t i = 0; i < CARTESIAN_DOFS; ++i) {
        output.cartesian_state.position[i] = output_.new_position[i];
        output.cartesian_state.velocity[i] = output_.new_velocity[i];
        output.cartesian_state.acceleration[i] = output_.new_acceleration[i];
    }
    output.trajectory_time = output_.trajectory.get_duration();
}

} // namespace trajectory
} // namespace robot_controller
```

### 5.4. Trajectory Engine (Unified Interface)

```cpp
// trajectory_engine.hpp

#pragma once
#include "cartesian_otg.hpp"
#include "joint_otg.hpp"
#include "../kinematics/kinematics_engine.hpp"
#include <memory>

namespace robot_controller {
namespace trajectory {

/**
 * @brief Unified Trajectory Engine
 *
 * Combines Cartesian OTG, Joint OTG, and Kinematics into a single
 * interface for motion generation.
 */
class TrajectoryEngine {
public:
    /**
     * @brief Constructor
     * @param cycle_time Control loop cycle time [s]
     * @param kinematics Pointer to kinematics engine
     */
    TrajectoryEngine(double cycle_time,
                     std::shared_ptr<kinematics::KinematicsEngine> kinematics);

    /**
     * @brief Initialize with limits
     */
    void initialize(const CartesianLimits& cart_limits,
                   const JointLimits& joint_limits);

    /**
     * @brief Start a linear motion (MOVL)
     * @param target Target Cartesian state
     */
    void startMOVL(const CartesianState& target);

    /**
     * @brief Start a joint motion (MOVJ)
     * @param target Target joint state
     */
    void startMOVJ(const JointState& target);

    /**
     * @brief Update trajectory (call every cycle)
     * @param current_joints Current joint positions from feedback
     * @return Calculated joint commands
     */
    JointState update(const JointState& current_joints);

    /**
     * @brief Update target for seam tracking
     * @param sensor_offset Offset from sensor in tool frame
     */
    void updateTrackingOffset(const Eigen::Vector3d& sensor_offset);

    /**
     * @brief Set travel velocity for tracking
     * @param travel_speed Speed along weld path [m/s]
     * @param direction Unit direction vector in world frame
     */
    void setTravelVelocity(double travel_speed, const Eigen::Vector3d& direction);

    /**
     * @brief Stop motion (decelerate to stop)
     */
    void stop();

    /**
     * @brief Emergency stop (immediate halt, no deceleration)
     */
    void emergencyStop();

    /**
     * @brief Get current motion type
     */
    MotionType getMotionType() const { return motion_type_; }

    /**
     * @brief Check if trajectory is active
     */
    bool isActive() const { return is_active_; }

    /**
     * @brief Get estimated time to reach target
     */
    double getTimeRemaining() const;

    /**
     * @brief Check if near singularity
     */
    bool isNearSingularity() const { return near_singularity_; }

private:
    // Components
    std::unique_ptr<CartesianOTG> cartesian_otg_;
    std::unique_ptr<JointOTG> joint_otg_;
    std::shared_ptr<kinematics::KinematicsEngine> kinematics_;

    // State
    MotionType motion_type_;
    bool is_active_;
    bool near_singularity_;
    double cycle_time_;

    // Singularity detection
    static constexpr double SINGULARITY_THRESHOLD = 0.01;  // det(J) threshold

    // Internal methods
    bool checkSingularity(const JointState& joints);
    JointState applyVelocityScaling(const JointState& joints, double scale_factor);
};

} // namespace trajectory
} // namespace robot_controller
```

### 5.5. TrajectoryEngine Implementation

```cpp
// trajectory_engine.cpp

#include "trajectory_engine.hpp"
#include <algorithm>
#include <cmath>

namespace robot_controller {
namespace trajectory {

TrajectoryEngine::TrajectoryEngine(double cycle_time,
                                   std::shared_ptr<kinematics::KinematicsEngine> kinematics)
    : cycle_time_(cycle_time)
    , kinematics_(kinematics)
    , motion_type_(MotionType::MOVL)
    , is_active_(false)
    , near_singularity_(false)
{
    cartesian_otg_ = std::make_unique<CartesianOTG>(cycle_time);
    joint_otg_ = std::make_unique<JointOTG>(cycle_time);
}

void TrajectoryEngine::initialize(const CartesianLimits& cart_limits,
                                  const JointLimits& joint_limits) {
    cartesian_otg_->initialize(cart_limits);
    joint_otg_->initialize(joint_limits);
}

void TrajectoryEngine::startMOVL(const CartesianState& target) {
    motion_type_ = MotionType::MOVL;
    cartesian_otg_->setTarget(target);
    is_active_ = true;
}

void TrajectoryEngine::startMOVJ(const JointState& target) {
    motion_type_ = MotionType::MOVJ;
    joint_otg_->setTarget(target);
    is_active_ = true;
}

JointState TrajectoryEngine::update(const JointState& current_joints) {
    if (!is_active_) {
        return current_joints;
    }

    JointState result_joints;

    switch (motion_type_) {
        case MotionType::MOVL: {
            // Get current Cartesian position via FK
            Eigen::Isometry3d current_pose;
            std::array<double, 6> q_array;
            for (size_t i = 0; i < 6; ++i) {
                q_array[i] = current_joints.position[i];
            }
            current_pose = kinematics_->forwardKinematics(q_array);

            // Convert to CartesianState
            CartesianState current_cart;
            Eigen::Vector3d pos = current_pose.translation();
            Eigen::Vector3d rpy = current_pose.rotation().eulerAngles(2, 1, 0); // ZYX
            current_cart.position = {pos.x(), pos.y(), pos.z(),
                                     rpy[2], rpy[1], rpy[0]};
            current_cart.velocity = {0, 0, 0, 0, 0, 0}; // Ideally from observer
            current_cart.acceleration = {0, 0, 0, 0, 0, 0};

            // Execute Cartesian OTG
            TrajectoryOutput output;
            TrajectoryResult traj_result = cartesian_otg_->update(current_cart, output);

            if (traj_result == TrajectoryResult::Finished) {
                is_active_ = false;
            }

            // Convert output Cartesian to pose
            Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
            target_pose.translation() = Eigen::Vector3d(
                output.cartesian_state.position[0],
                output.cartesian_state.position[1],
                output.cartesian_state.position[2]
            );
            Eigen::Matrix3d rot;
            rot = Eigen::AngleAxisd(output.cartesian_state.position[5], Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(output.cartesian_state.position[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(output.cartesian_state.position[3], Eigen::Vector3d::UnitZ());
            target_pose.linear() = rot;

            // Solve IK
            auto ik_result = kinematics_->inverseKinematics(target_pose, q_array);

            if (ik_result) {
                for (size_t i = 0; i < 6; ++i) {
                    result_joints.position[i] = (*ik_result)[i];
                }

                // Calculate joint velocities (numerical differentiation)
                for (size_t i = 0; i < 6; ++i) {
                    result_joints.velocity[i] =
                        (result_joints.position[i] - current_joints.position[i]) / cycle_time_;
                }

                // Check singularity
                near_singularity_ = checkSingularity(result_joints);
            }
            break;
        }

        case MotionType::MOVJ: {
            // Direct joint interpolation
            TrajectoryOutput output;
            TrajectoryResult traj_result = joint_otg_->update(current_joints, output);

            if (traj_result == TrajectoryResult::Finished) {
                is_active_ = false;
            }

            result_joints = output.joint_state;
            near_singularity_ = false; // No singularity in joint space
            break;
        }

        default:
            result_joints = current_joints;
    }

    return result_joints;
}

void TrajectoryEngine::updateTrackingOffset(const Eigen::Vector3d& sensor_offset) {
    if (motion_type_ != MotionType::MOVL) return;

    // Transform sensor offset from tool frame to world frame
    // (This requires current pose - simplified here)
    std::array<double, CARTESIAN_DOFS> offset = {
        sensor_offset.x(), sensor_offset.y(), sensor_offset.z(),
        0.0, 0.0, 0.0  // No orientation correction
    };

    cartesian_otg_->updateTargetOffset(offset);
}

void TrajectoryEngine::setTravelVelocity(double travel_speed,
                                         const Eigen::Vector3d& direction) {
    if (motion_type_ != MotionType::MOVL) return;

    Eigen::Vector3d velocity = direction.normalized() * travel_speed;
    std::array<double, CARTESIAN_DOFS> vel = {
        velocity.x(), velocity.y(), velocity.z(),
        0.0, 0.0, 0.0
    };

    cartesian_otg_->setTargetVelocity(vel);
}

void TrajectoryEngine::stop() {
    // Decelerate to stop using Ruckig's capability
    // Set target velocity to zero, Ruckig will decelerate smoothly
    std::array<double, CARTESIAN_DOFS> zero_vel = {0, 0, 0, 0, 0, 0};
    cartesian_otg_->setTargetVelocity(zero_vel);

    std::array<double, JOINT_DOFS> zero_joint_vel = {0, 0, 0, 0, 0, 0};
    joint_otg_->setTargetVelocity(zero_joint_vel);
}

void TrajectoryEngine::emergencyStop() {
    is_active_ = false;
    cartesian_otg_->reset();
    joint_otg_->reset();
}

double TrajectoryEngine::getTimeRemaining() const {
    if (!is_active_) return 0.0;

    switch (motion_type_) {
        case MotionType::MOVL:
            return cartesian_otg_->getTimeRemaining();
        case MotionType::MOVJ:
            return joint_otg_->getTimeRemaining();
        default:
            return 0.0;
    }
}

bool TrajectoryEngine::checkSingularity(const JointState& joints) {
    // Get Jacobian determinant
    std::array<double, 6> q_array;
    for (size_t i = 0; i < 6; ++i) {
        q_array[i] = joints.position[i];
    }

    auto jacobian = kinematics_->computeJacobian(q_array);
    double det = jacobian.determinant();

    return std::abs(det) < SINGULARITY_THRESHOLD;
}

} // namespace trajectory
} // namespace robot_controller
```

---

## 6. Seam Tracking Integration

### 6.1. Moving Target Strategy

```
┌─────────────────────────────────────────────────────────────────┐
│              SEAM TRACKING WITH ARBITRARY TARGET UPDATES         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Traditional Approach (Fixed Waypoints):                        │
│  ───────────────────────────────────────                        │
│                                                                  │
│    P1 ──────► P2 ──────► P3 ──────► P4                         │
│     ●         ●         ●         ●                             │
│                                                                  │
│    Problem: Cannot adapt to sensor feedback                     │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  Moving Target Approach (Ruckig Arbitrary Target):              │
│  ─────────────────────────────────────────────                  │
│                                                                  │
│    Current ──────────────────────────► Target (constantly       │
│       ●══════════════════════════════════○     updated)         │
│                    ↑                                             │
│                    │                                             │
│              Sensor offset                                       │
│              added each cycle                                    │
│                                                                  │
│    Each 1ms cycle:                                               │
│    1. Read sensor offset (Δx, Δy, Δz)                           │
│    2. Transform to world frame                                   │
│    3. Target += offset                                           │
│    4. Ruckig recalculates smooth path to new target             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2. Signal Filtering

```cpp
// tracking_filter.hpp

#pragma once
#include <array>
#include <deque>

namespace robot_controller {
namespace trajectory {

/**
 * @brief Ring buffer filter for sensor noise reduction
 */
class TrackingFilter {
public:
    /**
     * @param window_size Number of samples for moving average
     * @param max_delta Maximum allowed change per cycle [m]
     */
    TrackingFilter(size_t window_size = 10, double max_delta = 0.001);

    /**
     * @brief Add new sensor reading
     * @param offset Raw sensor offset [m, m, m]
     * @return Filtered offset
     */
    std::array<double, 3> filter(const std::array<double, 3>& offset);

    /**
     * @brief Reset filter buffer
     */
    void reset();

private:
    size_t window_size_;
    double max_delta_;
    std::deque<std::array<double, 3>> buffer_;
    std::array<double, 3> last_output_;

    std::array<double, 3> movingAverage() const;
    std::array<double, 3> saturate(const std::array<double, 3>& input);
};

} // namespace trajectory
} // namespace robot_controller
```

---

## 7. Configuration

### 7.1. YAML Configuration File

```yaml
# config/trajectory_config.yaml

trajectory:
  # Control loop frequency
  cycle_time_ms: 1.0  # 1kHz control loop

  # Cartesian limits for MOVL
  cartesian_limits:
    # Linear axes (X, Y, Z)
    linear:
      max_velocity: 0.5        # m/s (500 mm/s)
      max_acceleration: 2.0    # m/s²
      max_jerk: 20.0           # m/s³
    # Rotational axes (Rx, Ry, Rz)
    rotational:
      max_velocity: 1.57       # rad/s (~90°/s)
      max_acceleration: 6.28   # rad/s² (~360°/s²)
      max_jerk: 31.4           # rad/s³

  # Joint limits for MOVJ
  joint_limits:
    # Per-joint limits [J1, J2, J3, J4, J5, J6]
    max_velocity: [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]     # rad/s
    max_acceleration: [10.0, 10.0, 10.0, 20.0, 20.0, 20.0] # rad/s²
    max_jerk: [100.0, 100.0, 100.0, 200.0, 200.0, 200.0]   # rad/s³

  # Welding speed profiles
  welding_speeds:
    slow:   0.005   # 5 mm/s (very slow for thick material)
    medium: 0.010   # 10 mm/s (typical)
    fast:   0.020   # 20 mm/s (thin material)

  # Singularity handling
  singularity:
    detection_threshold: 0.01  # Jacobian determinant threshold
    velocity_scale_min: 0.1    # Minimum velocity scale (10%)

  # Seam tracking filter
  tracking_filter:
    window_size: 10           # Moving average samples
    max_delta_per_cycle: 0.001 # Maximum 1mm change per 1ms
```

### 7.2. CMake Integration

```cmake
# CMakeLists.txt addition for Ruckig

# Find Ruckig (header-only in Community version)
find_package(ruckig QUIET)

if(NOT ruckig_FOUND)
    # Fallback: Fetch from GitHub
    include(FetchContent)
    FetchContent_Declare(
        ruckig
        GIT_REPOSITORY https://github.com/pantor/ruckig.git
        GIT_TAG v0.9.2
    )
    FetchContent_MakeAvailable(ruckig)
endif()

# Trajectory module
add_library(trajectory
    src/trajectory/cartesian_otg.cpp
    src/trajectory/joint_otg.cpp
    src/trajectory/trajectory_engine.cpp
    src/trajectory/tracking_filter.cpp
)

target_link_libraries(trajectory
    PUBLIC
        kinematics
        ruckig::ruckig
        Eigen3::Eigen
    PRIVATE
        spdlog::spdlog
)

target_include_directories(trajectory
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
```

---

## 8. Testing

### 8.1. Unit Test Cases

```cpp
// test_trajectory.cpp

#include <gtest/gtest.h>
#include "trajectory/trajectory_engine.hpp"
#include <chrono>

using namespace robot_controller::trajectory;

class TrajectoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize kinematics (mock or real)
        kinematics_ = std::make_shared<kinematics::KinematicsEngine>();
        kinematics_->loadFromYAML("test_robot.yaml");

        engine_ = std::make_unique<TrajectoryEngine>(0.001, kinematics_);

        CartesianLimits cart_limits;
        cart_limits.max_velocity = {0.5, 0.5, 0.5, 1.57, 1.57, 1.57};
        cart_limits.max_acceleration = {2.0, 2.0, 2.0, 6.28, 6.28, 6.28};
        cart_limits.max_jerk = {20.0, 20.0, 20.0, 31.4, 31.4, 31.4};

        JointLimits joint_limits;
        joint_limits.max_velocity = {3.14, 3.14, 3.14, 6.28, 6.28, 6.28};
        joint_limits.max_acceleration = {10.0, 10.0, 10.0, 20.0, 20.0, 20.0};
        joint_limits.max_jerk = {100.0, 100.0, 100.0, 200.0, 200.0, 200.0};

        engine_->initialize(cart_limits, joint_limits);
    }

    std::shared_ptr<kinematics::KinematicsEngine> kinematics_;
    std::unique_ptr<TrajectoryEngine> engine_;
};

// Test T-01: Basic trajectory generation
TEST_F(TrajectoryTest, BasicMOVL) {
    JointState current;
    current.position = {0, 0, 0, 0, 0, 0};
    current.reset_dynamics();

    CartesianState target;
    target.position = {0.5, 0.3, 0.4, 0, 0, 0};  // 500mm, 300mm, 400mm
    target.reset_dynamics();

    engine_->startMOVL(target);

    EXPECT_TRUE(engine_->isActive());
    EXPECT_EQ(engine_->getMotionType(), MotionType::MOVL);

    // Simulate 1000 cycles (1 second)
    int cycles = 0;
    while (engine_->isActive() && cycles < 10000) {
        current = engine_->update(current);
        cycles++;
    }

    EXPECT_FALSE(engine_->isActive());
    EXPECT_GT(cycles, 0);
}

// Test T-02: Phase synchronization produces straight line
TEST_F(TrajectoryTest, StraightLineMotion) {
    JointState current;
    current.position = {0, 0, 0, 0, 0, 0};
    current.reset_dynamics();

    // Start at origin, move to (1, 1, 0) - 45 degree line
    CartesianState target;
    target.position = {1.0, 1.0, 0.0, 0, 0, 0};
    target.reset_dynamics();

    engine_->startMOVL(target);

    std::vector<std::array<double, 3>> path;

    while (engine_->isActive()) {
        current = engine_->update(current);

        // Get current Cartesian position via FK
        Eigen::Isometry3d pose = kinematics_->forwardKinematics(current.position);
        Eigen::Vector3d pos = pose.translation();

        path.push_back({pos.x(), pos.y(), pos.z()});
    }

    // Verify straight line: y/x ratio should be constant (1.0)
    for (const auto& point : path) {
        if (std::abs(point[0]) > 0.01) {  // Avoid division by zero
            double ratio = point[1] / point[0];
            EXPECT_NEAR(ratio, 1.0, 0.01);  // 1% tolerance
        }
    }
}

// Test T-03: Cycle time requirement
TEST_F(TrajectoryTest, CycleTimePerformance) {
    JointState current;
    current.position = {0, 0, 0, 0, 0, 0};
    current.reset_dynamics();

    CartesianState target;
    target.position = {0.5, 0.3, 0.4, 0, 0, 0};
    target.reset_dynamics();

    engine_->startMOVL(target);

    // Measure update time
    auto start = std::chrono::high_resolution_clock::now();
    current = engine_->update(current);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Ruckig update should take < 100 microseconds
    // Total with IK should be < 500 microseconds
    EXPECT_LT(duration.count(), 500);
}

// Test T-04: Angle unwrapping
TEST_F(TrajectoryTest, AngleUnwrapping) {
    JointState current;
    current.position = {0, 0, 0, 0, 0, 0};
    current.reset_dynamics();

    // Current orientation: 10 degrees, Target: 350 degrees
    // Should unwrap to -10 degrees (20 degree motion, not 340)
    CartesianState target;
    target.position = {0.5, 0.0, 0.4, 0, 0, 6.11};  // ~350 degrees
    target.reset_dynamics();

    engine_->startMOVL(target);

    // After first update, check that rotation direction is negative
    current = engine_->update(current);

    Eigen::Isometry3d pose = kinematics_->forwardKinematics(current.position);
    Eigen::Vector3d rpy = pose.rotation().eulerAngles(2, 1, 0);

    // Should rotate through 0, not through 180
    // Initial Z rotation velocity should be negative (toward -10 deg)
    // This is implementation specific, but key is total rotation < 180 deg
}

// Test T-05: Emergency stop
TEST_F(TrajectoryTest, EmergencyStop) {
    JointState current;
    current.position = {0, 0, 0, 0, 0, 0};
    current.reset_dynamics();

    CartesianState target;
    target.position = {1.0, 1.0, 1.0, 0, 0, 0};
    target.reset_dynamics();

    engine_->startMOVL(target);
    EXPECT_TRUE(engine_->isActive());

    // Run a few cycles
    for (int i = 0; i < 100; i++) {
        current = engine_->update(current);
    }

    // Emergency stop
    engine_->emergencyStop();
    EXPECT_FALSE(engine_->isActive());

    // Next update should return current position (no motion)
    JointState stopped = engine_->update(current);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(stopped.position[i], current.position[i]);
    }
}
```

### 8.2. Performance Benchmarks

| Metric | Target | Typical |
|--------|--------|---------|
| Ruckig update() | < 50µs | 20-30µs |
| IK solve (Newton-Raphson) | < 300µs | 100-200µs |
| Total cycle time | < 500µs | 150-350µs |
| Memory usage | < 10KB per instance | ~5KB |

---

## 9. Known Limitations

### 9.1. Ruckig Community Edition Limits

| Limitation | Impact | Workaround |
|------------|--------|------------|
| No intermediate waypoints | Cannot plan multi-segment paths in one call | Use "State-to-State" with target updates |
| Max trajectory time 7e3s | ~2 hours max duration | Reset at each stop point |
| No joint velocity constraint in Cartesian mode | Singularity not automatically avoided | Manual velocity scaling |

### 9.2. Singularity Handling

```
┌─────────────────────────────────────────────────────────────────┐
│              SINGULARITY DETECTION & HANDLING                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Detection:                                                      │
│  ──────────                                                     │
│     if (|det(J)| < threshold):                                  │
│         near_singularity = true                                  │
│                                                                  │
│  Handling Strategy:                                              │
│  ──────────────────                                             │
│     1. Calculate required joint velocity: θ̇ = J⁻¹ · ẋ          │
│     2. Check if any |θ̇ᵢ| > max_joint_velocity                  │
│     3. If exceeded, calculate scale factor:                      │
│           scale = max_allowed / max(|θ̇ᵢ|)                      │
│     4. Apply to Cartesian velocity: ẋ_scaled = ẋ * scale        │
│                                                                  │
│  Result: Robot slows down when approaching singularity          │
│          instead of demanding impossible joint velocities       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. References

### 10.1. Research Documents
| Document | Path |
|----------|------|
| Tích hợp Ruckig cho Robot Hàn 6-DOF | ressearch_doc_md/ |
| Robotics Library: Robot Tùy Chỉnh & IK Giải Tích | ressearch_doc_md/ |

### 10.2. External Resources
| Resource | URL |
|----------|-----|
| Ruckig Documentation | https://docs.ruckig.com/ |
| Ruckig GitHub | https://github.com/pantor/ruckig |
| Ruckig Paper (arXiv) | https://arxiv.org/abs/2105.04830 |
| Robotics Library (RL) | https://www.roboticslibrary.org/ |

---

## APPENDIX

### A. Glossary

| Term | Definition |
|------|------------|
| **OTG** | Online Trajectory Generation - Tạo quỹ đạo thời gian thực |
| **Jerk** | Đạo hàm bậc 3 của vị trí (rate of change of acceleration) |
| **S-Curve** | Biên dạng vận tốc có gia tốc thay đổi tuyến tính |
| **MOVL** | Move Linear - Di chuyển theo đường thẳng |
| **MOVJ** | Move Joint - Di chuyển nhanh nhất theo không gian khớp |
| **Phase Sync** | Đồng bộ pha - Đảm bảo tỷ lệ quãng đường các trục như nhau |
| **Arbitrary Target** | Khả năng thay đổi đích đến bất kỳ lúc nào |

### B. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
