# CORE_09: Simulation Mode

| Metadata      | Value                           |
|---------------|---------------------------------|
| Module ID     | CORE_09                         |
| Priority      | P1                              |
| Status        | DRAFT                           |
| Dependencies  | CORE_02, CORE_05, CORE_06, CORE_08 |

---

## 1. Tổng quan

### 1.1 Mục đích

Simulation Mode cho phép vận hành và test chương trình robot offline mà không cần kết nối hardware thực. Module này cung cấp VirtualController thay thế cho grblHAL firmware, mô phỏng đầy đủ hành vi robot bao gồm kinematics, dynamics, và I/O.

### 1.2 Use Cases

| Use Case | Mô tả |
|----------|-------|
| **Offline Programming** | Viết và test chương trình không cần robot thật |
| **Program Validation** | Kiểm tra collision, reachability trước khi chạy thật |
| **Training** | Đào tạo operator mà không có rủi ro |
| **Development** | Phát triển và debug software mà không cần hardware |
| **Demo** | Trình diễn cho khách hàng |

### 1.3 Simulation vs Real Mode

```
┌─────────────────────────────────────────────────────────────────┐
│                    MODE SWITCHING                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                    Core Logic (C++)                      │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │    │
│  │  │ StateManager │  │ Trajectory   │  │  Kinematics  │   │    │
│  │  └──────────────┘  └──────────────┘  └──────────────┘   │    │
│  │                          │                               │    │
│  │                    IMotionController                     │    │
│  │                          │                               │    │
│  │           ┌──────────────┴──────────────┐               │    │
│  │           │                             │               │    │
│  │           ▼                             ▼               │    │
│  │  ┌────────────────┐           ┌────────────────┐        │    │
│  │  │ grblHAL Driver │           │VirtualController│        │    │
│  │  │  (Real Mode)   │           │ (Simulation)   │        │    │
│  │  └───────┬────────┘           └───────┬────────┘        │    │
│  └──────────┼────────────────────────────┼─────────────────┘    │
│             │                            │                       │
│             ▼                            ▼                       │
│      ┌────────────┐               ┌────────────┐                │
│      │  Teensy    │               │  Software  │                │
│      │  Hardware  │               │  Emulation │                │
│      └────────────┘               └────────────┘                │
│                                                                  │
│      REAL MODE                    SIMULATION MODE                │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture

### 2.1 Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION SUBSYSTEM                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                   VirtualController                         │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │ │
│  │  │   Motion     │  │   Virtual    │  │   Timing     │      │ │
│  │  │  Simulator   │  │     I/O      │  │  Simulator   │      │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘      │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │ │
│  │  │  Collision   │  │   Dynamics   │  │   Welding    │      │ │
│  │  │  Detector    │  │  Simulator   │  │  Simulator   │      │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                    SimulationEngine                         │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │ │
│  │  │  Time Scale  │  │    State     │  │   Logging    │      │ │
│  │  │   Control    │  │   Snapshot   │  │  & Metrics   │      │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 IMotionController Interface

```cpp
// IMotionController.h
#pragma once
#include <array>
#include <string>
#include <functional>

namespace Motion {

struct JointState {
    std::array<double, 6> positions;      // rad
    std::array<double, 6> velocities;     // rad/s
    std::array<double, 6> accelerations;  // rad/s²
    std::array<double, 6> torques;        // Nm (estimated)
};

struct ControllerStatus {
    bool connected = false;
    bool enabled = false;
    bool moving = false;
    bool error = false;
    bool homed = false;

    std::string error_message;
    int error_code = 0;

    // Buffer status
    int buffer_available = 0;
    int buffer_used = 0;
};

struct DigitalIO {
    uint32_t inputs = 0;   // 32 digital inputs
    uint32_t outputs = 0;  // 32 digital outputs
};

struct AnalogIO {
    std::array<double, 8> inputs;   // 0-10V
    std::array<double, 8> outputs;  // 0-10V
};

// Abstract interface for motion controller
class IMotionController {
public:
    virtual ~IMotionController() = default;

    // Connection
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // Enable/Disable
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool isEnabled() const = 0;

    // Motion commands
    virtual bool sendGcode(const std::string& gcode) = 0;
    virtual bool jog(int axis, double velocity) = 0;
    virtual bool stopJog() = 0;
    virtual bool stop() = 0;
    virtual bool emergencyStop() = 0;

    // Homing
    virtual bool home(int axis = -1) = 0;  // -1 = all axes
    virtual bool isHomed() const = 0;

    // Status
    virtual JointState getJointState() const = 0;
    virtual ControllerStatus getStatus() const = 0;

    // I/O
    virtual DigitalIO getDigitalIO() const = 0;
    virtual AnalogIO getAnalogIO() const = 0;
    virtual bool setDigitalOutput(int pin, bool value) = 0;
    virtual bool setAnalogOutput(int channel, double value) = 0;

    // Callbacks
    using StatusCallback = std::function<void(const ControllerStatus&)>;
    using JointStateCallback = std::function<void(const JointState&)>;

    virtual void setStatusCallback(StatusCallback cb) = 0;
    virtual void setJointStateCallback(JointStateCallback cb) = 0;

    // Simulation-specific (no-op in real controller)
    virtual bool isSimulation() const = 0;
    virtual void setTimeScale(double scale) {}
    virtual void pause() {}
    virtual void resume() {}
};

} // namespace Motion
```

---

## 3. VirtualController

### 3.1 Implementation

```cpp
// VirtualController.h
#pragma once
#include "IMotionController.h"
#include "../Kinematics/KinematicsSolver.h"
#include "../Trajectory/TrajectoryPlanner.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>

namespace Motion {

struct SimulationConfig {
    // Timing
    double update_rate_hz = 1000.0;   // Internal simulation rate
    double time_scale = 1.0;          // 1.0 = real-time, 2.0 = 2x speed

    // Robot limits (from config)
    std::array<double, 6> max_velocity;      // rad/s
    std::array<double, 6> max_acceleration;  // rad/s²
    std::array<double, 6> max_jerk;          // rad/s³

    // Dynamics (optional)
    bool enable_dynamics = false;
    std::array<double, 6> joint_inertia;     // kg·m²
    std::array<double, 6> joint_friction;    // Nm·s/rad

    // Delays (simulate real hardware latency)
    double command_delay_ms = 1.0;
    double feedback_delay_ms = 0.5;
};

class VirtualController : public IMotionController {
public:
    explicit VirtualController(const SimulationConfig& config = {});
    ~VirtualController() override;

    // IMotionController implementation
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override { return connected_; }

    bool enable() override;
    bool disable() override;
    bool isEnabled() const override { return enabled_; }

    bool sendGcode(const std::string& gcode) override;
    bool jog(int axis, double velocity) override;
    bool stopJog() override;
    bool stop() override;
    bool emergencyStop() override;

    bool home(int axis = -1) override;
    bool isHomed() const override { return homed_; }

    JointState getJointState() const override;
    ControllerStatus getStatus() const override;

    DigitalIO getDigitalIO() const override;
    AnalogIO getAnalogIO() const override;
    bool setDigitalOutput(int pin, bool value) override;
    bool setAnalogOutput(int channel, double value) override;

    void setStatusCallback(StatusCallback cb) override { status_cb_ = cb; }
    void setJointStateCallback(JointStateCallback cb) override { joint_cb_ = cb; }

    // Simulation-specific
    bool isSimulation() const override { return true; }
    void setTimeScale(double scale) override;
    void pause() override;
    void resume() override;

    // Additional simulation features
    void reset();
    void setJointPositions(const std::array<double, 6>& positions);
    double getSimulationTime() const { return sim_time_; }

    // Collision callback
    using CollisionCallback = std::function<void(const std::string& info)>;
    void setCollisionCallback(CollisionCallback cb) { collision_cb_ = cb; }

private:
    void simulationLoop();
    void processGcodeQueue();
    void updateMotion(double dt);
    void updateDynamics(double dt);
    void checkLimits();

    // G-code parsing
    struct GcodeCommand {
        char type;           // G, M, etc.
        int code;            // G0, G1, M3, etc.
        std::map<char, double> params;  // X, Y, Z, F, etc.
    };
    GcodeCommand parseGcode(const std::string& line);
    void executeGcode(const GcodeCommand& cmd);

    SimulationConfig config_;

    // State
    std::atomic<bool> connected_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> homed_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};

    // Joint state
    mutable std::mutex state_mutex_;
    JointState current_state_;
    JointState target_state_;

    // I/O
    mutable std::mutex io_mutex_;
    DigitalIO digital_io_;
    AnalogIO analog_io_;

    // Motion
    std::array<double, 6> jog_velocities_{};
    bool is_jogging_ = false;

    // G-code queue
    std::mutex gcode_mutex_;
    std::queue<std::string> gcode_queue_;
    std::string current_gcode_;
    bool executing_gcode_ = false;

    // Timing
    double sim_time_ = 0.0;
    std::atomic<double> time_scale_{1.0};

    // Thread
    std::thread sim_thread_;

    // Callbacks
    StatusCallback status_cb_;
    JointStateCallback joint_cb_;
    CollisionCallback collision_cb_;
};

} // namespace Motion
```

### 3.2 Simulation Loop

```cpp
// VirtualController.cpp
#include "VirtualController.h"
#include <chrono>
#include <spdlog/spdlog.h>

namespace Motion {

VirtualController::VirtualController(const SimulationConfig& config)
    : config_(config) {

    // Initialize joint state
    current_state_.positions.fill(0.0);
    current_state_.velocities.fill(0.0);
    current_state_.accelerations.fill(0.0);
    current_state_.torques.fill(0.0);

    target_state_ = current_state_;

    // Initialize I/O
    analog_io_.inputs.fill(0.0);
    analog_io_.outputs.fill(0.0);
}

VirtualController::~VirtualController() {
    disconnect();
}

bool VirtualController::connect() {
    if (connected_) return true;

    connected_ = true;
    running_ = true;

    // Start simulation thread
    sim_thread_ = std::thread(&VirtualController::simulationLoop, this);

    spdlog::info("VirtualController connected (Simulation Mode)");
    return true;
}

void VirtualController::disconnect() {
    if (!connected_) return;

    running_ = false;
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }

    connected_ = false;
    enabled_ = false;

    spdlog::info("VirtualController disconnected");
}

void VirtualController::simulationLoop() {
    using clock = std::chrono::high_resolution_clock;

    const double dt = 1.0 / config_.update_rate_hz;
    auto next_time = clock::now();

    while (running_) {
        if (!paused_ && enabled_) {
            // Process G-code queue
            processGcodeQueue();

            // Update motion
            updateMotion(dt * time_scale_);

            // Update dynamics (optional)
            if (config_.enable_dynamics) {
                updateDynamics(dt * time_scale_);
            }

            // Check limits
            checkLimits();

            // Update simulation time
            sim_time_ += dt * time_scale_;

            // Callbacks
            if (joint_cb_) {
                joint_cb_(getJointState());
            }
        }

        // Sleep until next update
        next_time += std::chrono::microseconds(
            static_cast<int64_t>(1e6 / config_.update_rate_hz));
        std::this_thread::sleep_until(next_time);
    }
}

void VirtualController::updateMotion(double dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (is_jogging_) {
        // Jog mode: apply velocity directly
        for (int i = 0; i < 6; i++) {
            current_state_.velocities[i] = jog_velocities_[i];
            current_state_.positions[i] += jog_velocities_[i] * dt;
        }
    } else if (executing_gcode_) {
        // Trajectory following
        for (int i = 0; i < 6; i++) {
            double error = target_state_.positions[i] - current_state_.positions[i];

            // Simple P controller with velocity limit
            double vel = error * 10.0;  // P gain
            vel = std::clamp(vel,
                -config_.max_velocity[i],
                config_.max_velocity[i]);

            // Apply acceleration limit
            double acc = (vel - current_state_.velocities[i]) / dt;
            acc = std::clamp(acc,
                -config_.max_acceleration[i],
                config_.max_acceleration[i]);

            current_state_.velocities[i] += acc * dt;
            current_state_.positions[i] += current_state_.velocities[i] * dt;
            current_state_.accelerations[i] = acc;
        }

        // Check if motion complete
        bool complete = true;
        for (int i = 0; i < 6; i++) {
            if (std::abs(target_state_.positions[i] - current_state_.positions[i]) > 0.0001) {
                complete = false;
                break;
            }
        }

        if (complete) {
            executing_gcode_ = false;
            current_state_.velocities.fill(0.0);
            current_state_.accelerations.fill(0.0);
        }
    }
}

bool VirtualController::sendGcode(const std::string& gcode) {
    if (!connected_ || !enabled_) return false;

    std::lock_guard<std::mutex> lock(gcode_mutex_);
    gcode_queue_.push(gcode);

    spdlog::debug("[SIM] Queued G-code: {}", gcode);
    return true;
}

void VirtualController::processGcodeQueue() {
    if (executing_gcode_) return;

    std::lock_guard<std::mutex> lock(gcode_mutex_);
    if (gcode_queue_.empty()) return;

    current_gcode_ = gcode_queue_.front();
    gcode_queue_.pop();

    auto cmd = parseGcode(current_gcode_);
    executeGcode(cmd);
}

void VirtualController::executeGcode(const GcodeCommand& cmd) {
    if (cmd.type == 'G') {
        switch (cmd.code) {
            case 0:   // G0 - Rapid move
            case 1: { // G1 - Linear move
                // Extract joint positions (A, B, C, U, V, W for 6 axes)
                // Or Cartesian (X, Y, Z, A, B, C) - need IK
                std::lock_guard<std::mutex> lock(state_mutex_);

                // Assuming joint mode with A-F for J1-J6
                if (cmd.params.count('A')) target_state_.positions[0] = cmd.params.at('A') * M_PI / 180.0;
                if (cmd.params.count('B')) target_state_.positions[1] = cmd.params.at('B') * M_PI / 180.0;
                if (cmd.params.count('C')) target_state_.positions[2] = cmd.params.at('C') * M_PI / 180.0;
                if (cmd.params.count('U')) target_state_.positions[3] = cmd.params.at('U') * M_PI / 180.0;
                if (cmd.params.count('V')) target_state_.positions[4] = cmd.params.at('V') * M_PI / 180.0;
                if (cmd.params.count('W')) target_state_.positions[5] = cmd.params.at('W') * M_PI / 180.0;

                executing_gcode_ = true;
                break;
            }

            case 4: { // G4 - Dwell
                double dwell_s = cmd.params.count('P') ? cmd.params.at('P') : 0.0;
                // Sleep in simulation time
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(dwell_s * 1000 / time_scale_)));
                break;
            }

            case 28: { // G28 - Home
                std::lock_guard<std::mutex> lock(state_mutex_);
                target_state_.positions.fill(0.0);
                executing_gcode_ = true;
                homed_ = true;
                break;
            }
        }
    } else if (cmd.type == 'M') {
        switch (cmd.code) {
            case 3: { // M3 - Spindle/Welding ON
                std::lock_guard<std::mutex> lock(io_mutex_);
                digital_io_.outputs |= (1 << 0);  // Set DO0
                spdlog::debug("[SIM] Weld ON");
                break;
            }

            case 5: { // M5 - Spindle/Welding OFF
                std::lock_guard<std::mutex> lock(io_mutex_);
                digital_io_.outputs &= ~(1 << 0);  // Clear DO0
                spdlog::debug("[SIM] Weld OFF");
                break;
            }

            case 62: { // M62 - Set digital output
                int pin = cmd.params.count('P') ? static_cast<int>(cmd.params.at('P')) : 0;
                setDigitalOutput(pin, true);
                break;
            }

            case 63: { // M63 - Clear digital output
                int pin = cmd.params.count('P') ? static_cast<int>(cmd.params.at('P')) : 0;
                setDigitalOutput(pin, false);
                break;
            }
        }
    }
}

bool VirtualController::jog(int axis, double velocity) {
    if (!enabled_ || axis < 0 || axis >= 6) return false;

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Clamp velocity to limits
    velocity = std::clamp(velocity,
        -config_.max_velocity[axis],
        config_.max_velocity[axis]);

    jog_velocities_[axis] = velocity;
    is_jogging_ = true;

    spdlog::debug("[SIM] Jog axis {} at {} rad/s", axis, velocity);
    return true;
}

bool VirtualController::stopJog() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    jog_velocities_.fill(0.0);
    is_jogging_ = false;
    return true;
}

bool VirtualController::stop() {
    stopJog();

    std::lock_guard<std::mutex> lock(gcode_mutex_);
    while (!gcode_queue_.empty()) gcode_queue_.pop();
    executing_gcode_ = false;

    {
        std::lock_guard<std::mutex> lock2(state_mutex_);
        target_state_ = current_state_;
        current_state_.velocities.fill(0.0);
        current_state_.accelerations.fill(0.0);
    }

    return true;
}

bool VirtualController::emergencyStop() {
    stop();
    enabled_ = false;

    spdlog::warn("[SIM] Emergency Stop activated");
    return true;
}

} // namespace Motion
```

---

## 4. Motion Simulator

### 4.1 Trajectory Simulation

```cpp
// MotionSimulator.h
#pragma once
#include "VirtualController.h"
#include "../Trajectory/TrajectoryPlanner.h"

namespace Motion {

class MotionSimulator {
public:
    MotionSimulator(VirtualController& controller,
                    Trajectory::TrajectoryPlanner& planner);

    // Execute trajectory in simulation
    struct SimulationResult {
        bool success = false;
        double total_time_s = 0.0;
        double max_velocity = 0.0;
        double max_acceleration = 0.0;
        std::vector<JointState> trajectory_log;
        std::string error_message;
    };

    SimulationResult simulateTrajectory(
        const std::vector<Trajectory::Waypoint>& waypoints);

    // Dry run - calculate without executing
    SimulationResult dryRun(
        const std::vector<Trajectory::Waypoint>& waypoints);

    // Record simulation data
    void startRecording();
    void stopRecording();
    std::vector<JointState> getRecordedData() const;

private:
    VirtualController& controller_;
    Trajectory::TrajectoryPlanner& planner_;

    bool recording_ = false;
    std::vector<JointState> recorded_data_;
};

} // namespace Motion
```

### 4.2 Cycle Time Estimation

```cpp
// CycleTimeEstimator.h
#pragma once
#include <vector>

namespace Motion {

struct CycleTimeBreakdown {
    double motion_time_s = 0.0;      // Robot movement
    double process_time_s = 0.0;     // Welding, gripper, etc.
    double dwell_time_s = 0.0;       // G4 dwells
    double io_time_s = 0.0;          // I/O operations

    double total() const {
        return motion_time_s + process_time_s + dwell_time_s + io_time_s;
    }
};

class CycleTimeEstimator {
public:
    // Estimate from G-code program
    CycleTimeBreakdown estimate(const std::string& program);

    // Estimate from waypoints
    CycleTimeBreakdown estimate(
        const std::vector<Trajectory::Waypoint>& waypoints,
        double travel_speed_mm_s,
        double process_speed_mm_s);

private:
    double calculateMoveTime(
        const std::array<double, 6>& start,
        const std::array<double, 6>& end,
        double max_speed);
};

} // namespace Motion
```

---

## 5. Collision Detection

### 5.1 Collision Detector

```cpp
// CollisionDetector.h
#pragma once
#include "../Kinematics/KinematicsSolver.h"
#include <vector>
#include <memory>

namespace Simulation {

struct CollisionObject {
    enum class Type { BOX, CYLINDER, SPHERE, MESH };

    Type type;
    std::string name;

    // Transform
    Eigen::Matrix4d transform;

    // Dimensions based on type
    Eigen::Vector3d dimensions;  // For BOX: x, y, z
    double radius = 0;           // For CYLINDER, SPHERE
    double height = 0;           // For CYLINDER

    // Mesh data (if type == MESH)
    std::vector<Eigen::Vector3f> vertices;
    std::vector<uint32_t> indices;
};

struct CollisionResult {
    bool collision = false;
    std::string object1_name;
    std::string object2_name;
    Eigen::Vector3d contact_point;
    double penetration_depth = 0;
};

class CollisionDetector {
public:
    // Robot model
    void setRobotModel(const std::vector<CollisionObject>& links);

    // Environment
    void addObject(const CollisionObject& object);
    void removeObject(const std::string& name);
    void clearEnvironment();

    // Self-collision zones (pairs of links that can collide)
    void setSelfCollisionPairs(const std::vector<std::pair<int, int>>& pairs);

    // Check collision for given joint state
    CollisionResult checkCollision(const std::array<double, 6>& joint_positions);

    // Check entire trajectory
    std::vector<CollisionResult> checkTrajectory(
        const std::vector<std::array<double, 6>>& trajectory,
        int sample_rate = 10);  // Check every N points

    // Get minimum distance to collision
    double getMinDistance(const std::array<double, 6>& joint_positions);

private:
    // Update robot link transforms from joint positions
    void updateRobotTransforms(const std::array<double, 6>& positions);

    // Primitive collision checks
    bool checkBoxBox(const CollisionObject& a, const CollisionObject& b);
    bool checkSphereSphere(const CollisionObject& a, const CollisionObject& b);
    bool checkBoxSphere(const CollisionObject& box, const CollisionObject& sphere);

    std::vector<CollisionObject> robot_links_;
    std::vector<CollisionObject> environment_;
    std::vector<std::pair<int, int>> self_collision_pairs_;

    std::shared_ptr<Kinematics::KinematicsSolver> kinematics_;
};

} // namespace Simulation
```

### 5.2 Reachability Analyzer

```cpp
// ReachabilityAnalyzer.h
#pragma once
#include "../Kinematics/KinematicsSolver.h"

namespace Simulation {

struct ReachabilityResult {
    bool reachable = false;
    bool has_solution = false;
    bool within_limits = false;
    bool collision_free = false;

    std::array<double, 6> joint_solution;
    std::string failure_reason;
};

class ReachabilityAnalyzer {
public:
    ReachabilityAnalyzer(
        Kinematics::KinematicsSolver& kinematics,
        CollisionDetector& collision);

    // Check single pose
    ReachabilityResult checkPose(const Kinematics::CartesianPose& pose);

    // Check all poses in program
    std::vector<ReachabilityResult> checkProgram(
        const std::vector<Kinematics::CartesianPose>& poses);

    // Generate reachability map (for visualization)
    struct ReachabilityMap {
        std::vector<Eigen::Vector3f> reachable_points;
        std::vector<Eigen::Vector3f> unreachable_points;
        Eigen::Vector3f workspace_min;
        Eigen::Vector3f workspace_max;
    };

    ReachabilityMap generateMap(
        double resolution_mm = 50.0,
        const Eigen::Vector3f& center = Eigen::Vector3f::Zero());

private:
    Kinematics::KinematicsSolver& kinematics_;
    CollisionDetector& collision_;
};

} // namespace Simulation
```

---

## 6. Virtual I/O

### 6.1 I/O Simulator

```cpp
// VirtualIO.h
#pragma once
#include <functional>
#include <map>
#include <mutex>

namespace Simulation {

class VirtualIO {
public:
    // Digital I/O
    bool getDigitalInput(int pin) const;
    void setDigitalInput(int pin, bool value);  // For simulation injection
    bool getDigitalOutput(int pin) const;
    void setDigitalOutput(int pin, bool value);

    // Analog I/O
    double getAnalogInput(int channel) const;
    void setAnalogInput(int channel, double value);  // 0-10V
    double getAnalogOutput(int channel) const;
    void setAnalogOutput(int channel, double value);

    // Simulation scenarios
    void simulateSensorTrigger(int pin, bool value);
    void simulateAnalogSensor(int channel, double value);

    // Linked I/O (output triggers input)
    void linkIO(int output_pin, int input_pin, uint32_t delay_ms = 0);
    void unlinkIO(int output_pin);

    // Callbacks
    using IOChangeCallback = std::function<void(int pin, bool value)>;
    void setDigitalInputCallback(IOChangeCallback cb) { di_cb_ = cb; }
    void setDigitalOutputCallback(IOChangeCallback cb) { do_cb_ = cb; }

private:
    mutable std::mutex mutex_;

    uint32_t digital_inputs_ = 0;
    uint32_t digital_outputs_ = 0;
    std::array<double, 8> analog_inputs_{};
    std::array<double, 8> analog_outputs_{};

    // Linked pairs: output_pin -> (input_pin, delay_ms)
    std::map<int, std::pair<int, uint32_t>> linked_io_;

    IOChangeCallback di_cb_;
    IOChangeCallback do_cb_;
};

} // namespace Simulation
```

### 6.2 Welding Simulator

```cpp
// WeldingSimulator.h
#pragma once
#include "VirtualIO.h"

namespace Simulation {

struct WeldingSimState {
    bool arc_on = false;
    bool gas_flowing = false;
    bool wire_feeding = false;

    double current_amps = 0.0;
    double voltage_volts = 0.0;
    double wire_speed_m_min = 0.0;
    double gas_flow_l_min = 0.0;

    // Timing
    double arc_on_time_s = 0.0;
    double total_weld_length_mm = 0.0;
};

class WeldingSimulator {
public:
    explicit WeldingSimulator(VirtualIO& io);

    // Update simulation (call from main loop)
    void update(double dt, const Eigen::Vector3d& tcp_position);

    // Get current state
    WeldingSimState getState() const { return state_; }

    // Set parameters (from analog outputs)
    void setWeldParameters(double current, double voltage, double wire_speed);

    // Events
    void onArcStart();
    void onArcStop();

    // Simulate faults
    void simulateArcFault();
    void simulateWireFault();
    void simulateGasFault();

private:
    VirtualIO& io_;
    WeldingSimState state_;

    Eigen::Vector3d last_tcp_position_;
};

} // namespace Simulation
```

---

## 7. Simulation Engine

### 7.1 Engine Core

```cpp
// SimulationEngine.h
#pragma once
#include "VirtualController.h"
#include "CollisionDetector.h"
#include "VirtualIO.h"
#include "WeldingSimulator.h"

namespace Simulation {

enum class SimulationState {
    STOPPED,
    RUNNING,
    PAUSED,
    ERROR
};

struct SimulationMetrics {
    double simulation_time_s = 0.0;
    double real_time_s = 0.0;
    double time_ratio = 1.0;  // sim_time / real_time

    int gcode_executed = 0;
    int collisions_detected = 0;
    double total_distance_mm = 0.0;
};

class SimulationEngine {
public:
    SimulationEngine();

    // Components
    VirtualController& getController() { return controller_; }
    CollisionDetector& getCollisionDetector() { return collision_; }
    VirtualIO& getIO() { return io_; }
    WeldingSimulator& getWeldingSim() { return welding_sim_; }

    // Control
    void start();
    void stop();
    void pause();
    void resume();
    void reset();

    // Time control
    void setTimeScale(double scale);
    double getTimeScale() const;

    // State
    SimulationState getState() const { return state_; }
    SimulationMetrics getMetrics() const { return metrics_; }

    // Snapshot (save/restore state)
    struct Snapshot {
        std::array<double, 6> joint_positions;
        uint32_t digital_io;
        std::array<double, 8> analog_io;
        double simulation_time;
    };

    Snapshot takeSnapshot() const;
    void restoreSnapshot(const Snapshot& snapshot);

    // Load environment
    void loadEnvironment(const std::string& filepath);

private:
    Motion::SimulationConfig sim_config_;
    VirtualController controller_;
    CollisionDetector collision_;
    VirtualIO io_;
    WeldingSimulator welding_sim_;

    SimulationState state_ = SimulationState::STOPPED;
    SimulationMetrics metrics_;
};

} // namespace Simulation
```

---

## 8. HMI Integration

### 8.1 Simulation Panel

```xml
<!-- SimulationPanelView.xaml -->
<UserControl x:Class="RobotHMI.Views.SimulationPanelView">
    <Border Background="#2D2D30" CornerRadius="5" Padding="10">
        <StackPanel>
            <!-- Mode Indicator -->
            <Border Background="#FF5722" CornerRadius="3" Padding="5" Margin="0,0,0,10">
                <TextBlock Text="SIMULATION MODE"
                           Foreground="White"
                           FontWeight="Bold"
                           HorizontalAlignment="Center"/>
            </Border>

            <!-- Time Controls -->
            <GroupBox Header="Time Control" Margin="0,0,0,10">
                <StackPanel>
                    <StackPanel Orientation="Horizontal">
                        <TextBlock Text="Time Scale:" Width="80"/>
                        <Slider Value="{Binding TimeScale}"
                                Minimum="0.1" Maximum="10"
                                Width="150"/>
                        <TextBlock Text="{Binding TimeScale, StringFormat={}{0:F1}x}"
                                   Width="40"/>
                    </StackPanel>

                    <StackPanel Orientation="Horizontal" Margin="0,10,0,0">
                        <Button Content="▶" Command="{Binding PlayCommand}"
                                Width="40" Margin="0,0,5,0"/>
                        <Button Content="⏸" Command="{Binding PauseCommand}"
                                Width="40" Margin="0,0,5,0"/>
                        <Button Content="⏹" Command="{Binding StopCommand}"
                                Width="40" Margin="0,0,5,0"/>
                        <Button Content="↺" Command="{Binding ResetCommand}"
                                Width="40"/>
                    </StackPanel>
                </StackPanel>
            </GroupBox>

            <!-- Simulation Time -->
            <GroupBox Header="Simulation Time">
                <StackPanel>
                    <TextBlock Text="{Binding SimulationTime, StringFormat='Sim: {0:F2} s'}"
                               FontSize="16"/>
                    <TextBlock Text="{Binding RealTime, StringFormat='Real: {0:F2} s'}"
                               Foreground="Gray"/>
                </StackPanel>
            </GroupBox>

            <!-- Metrics -->
            <GroupBox Header="Metrics" Margin="0,10,0,0">
                <StackPanel>
                    <TextBlock Text="{Binding GcodeExecuted, StringFormat='G-codes: {0}'}"/>
                    <TextBlock Text="{Binding TotalDistance, StringFormat='Distance: {0:F1} mm'}"/>
                    <TextBlock Text="{Binding CollisionCount, StringFormat='Collisions: {0}'}"
                               Foreground="{Binding CollisionCountColor}"/>
                </StackPanel>
            </GroupBox>

            <!-- Virtual I/O -->
            <GroupBox Header="Virtual I/O" Margin="0,10,0,0">
                <Grid>
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>

                    <!-- Digital Inputs -->
                    <StackPanel Grid.Column="0">
                        <TextBlock Text="DI" FontWeight="Bold"/>
                        <ItemsControl ItemsSource="{Binding DigitalInputs}">
                            <ItemsControl.ItemTemplate>
                                <DataTemplate>
                                    <StackPanel Orientation="Horizontal">
                                        <CheckBox IsChecked="{Binding Value}"
                                                  IsEnabled="True"/>
                                        <TextBlock Text="{Binding Name}" Margin="5,0"/>
                                    </StackPanel>
                                </DataTemplate>
                            </ItemsControl.ItemTemplate>
                        </ItemsControl>
                    </StackPanel>

                    <!-- Digital Outputs -->
                    <StackPanel Grid.Column="1">
                        <TextBlock Text="DO" FontWeight="Bold"/>
                        <ItemsControl ItemsSource="{Binding DigitalOutputs}">
                            <ItemsControl.ItemTemplate>
                                <DataTemplate>
                                    <StackPanel Orientation="Horizontal">
                                        <Ellipse Width="10" Height="10"
                                                 Fill="{Binding StatusColor}"/>
                                        <TextBlock Text="{Binding Name}" Margin="5,0"/>
                                    </StackPanel>
                                </DataTemplate>
                            </ItemsControl.ItemTemplate>
                        </ItemsControl>
                    </StackPanel>
                </Grid>
            </GroupBox>

            <!-- Inject Events -->
            <GroupBox Header="Inject Events" Margin="0,10,0,0">
                <StackPanel>
                    <Button Content="Trigger Part Sensor"
                            Command="{Binding TriggerPartSensorCommand}"
                            Margin="0,0,0,5"/>
                    <Button Content="Simulate Arc Fault"
                            Command="{Binding SimulateArcFaultCommand}"
                            Margin="0,0,0,5"/>
                    <Button Content="Trigger E-Stop"
                            Command="{Binding TriggerEStopCommand}"/>
                </StackPanel>
            </GroupBox>
        </StackPanel>
    </Border>
</UserControl>
```

### 8.2 3D Visualization Integration

```cpp
// SimulationVisualizer.h
// Integration with Helix Toolkit for 3D visualization

namespace Simulation {

struct VisualizationConfig {
    bool show_collision_volumes = false;
    bool show_trajectory_preview = true;
    bool show_weld_trail = true;
    bool show_workspace_boundary = false;

    // Colors
    uint32_t robot_color = 0xFF5E00;       // KUKA Orange
    uint32_t ghost_robot_color = 0x808080;  // Gray
    uint32_t collision_color = 0xFF0000;    // Red
    uint32_t trajectory_color = 0x00FF00;   // Green
    uint32_t weld_color = 0xFFD700;         // Gold
};

class SimulationVisualizer {
public:
    // Update robot model from joint state
    void updateRobotPose(const std::array<double, 6>& joint_positions);

    // Show ghost robot at target position
    void showGhostRobot(const std::array<double, 6>& target_positions);
    void hideGhostRobot();

    // Trajectory preview
    void showTrajectoryPreview(const std::vector<Eigen::Vector3d>& points);
    void clearTrajectoryPreview();

    // Weld trail
    void addWeldPoint(const Eigen::Vector3d& point);
    void clearWeldTrail();

    // Collision visualization
    void highlightCollision(const CollisionResult& result);
    void clearCollisionHighlight();

    // Environment
    void addEnvironmentObject(const CollisionObject& obj);
    void removeEnvironmentObject(const std::string& name);

private:
    VisualizationConfig config_;
};

} // namespace Simulation
```

---

## 9. Configuration

### 9.1 Simulation Config File

```yaml
# config/simulation.yaml
simulation:
  # Timing
  update_rate_hz: 1000
  default_time_scale: 1.0

  # Robot dynamics (optional)
  enable_dynamics: false
  joint_inertia: [1.0, 1.0, 0.5, 0.1, 0.1, 0.05]  # kg·m²
  joint_friction: [0.1, 0.1, 0.05, 0.02, 0.02, 0.01]  # Nm·s/rad

  # Delays (simulate real hardware)
  command_delay_ms: 1.0
  feedback_delay_ms: 0.5

collision:
  enable: true
  check_self_collision: true
  check_environment: true

  # Self-collision pairs (link indices that can collide)
  self_collision_pairs:
    - [0, 4]  # Base vs Wrist 1
    - [0, 5]  # Base vs Wrist 2
    - [1, 4]  # Link 1 vs Wrist 1
    - [1, 5]  # Link 1 vs Wrist 2

visualization:
  show_collision_volumes: false
  show_trajectory_preview: true
  show_weld_trail: true
  show_workspace_boundary: false

  robot_color: 0xFF5E00
  ghost_robot_color: 0x808080

virtual_io:
  # Linked I/O (output triggers input after delay)
  links:
    - output: 0
      input: 0
      delay_ms: 50
    - output: 1
      input: 1
      delay_ms: 100
```

---

## 10. Testing

### 10.1 Unit Tests

```cpp
// test_simulation.cpp
#include <gtest/gtest.h>
#include "VirtualController.h"
#include "CollisionDetector.h"

class VirtualControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        Motion::SimulationConfig config;
        config.update_rate_hz = 100;  // Faster for tests
        controller_ = std::make_unique<Motion::VirtualController>(config);
    }

    std::unique_ptr<Motion::VirtualController> controller_;
};

TEST_F(VirtualControllerTest, ConnectDisconnect) {
    EXPECT_FALSE(controller_->isConnected());

    EXPECT_TRUE(controller_->connect());
    EXPECT_TRUE(controller_->isConnected());
    EXPECT_TRUE(controller_->isSimulation());

    controller_->disconnect();
    EXPECT_FALSE(controller_->isConnected());
}

TEST_F(VirtualControllerTest, JogMotion) {
    controller_->connect();
    controller_->enable();

    auto initial_state = controller_->getJointState();

    // Jog axis 0 at 0.1 rad/s
    controller_->jog(0, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controller_->stopJog();

    auto final_state = controller_->getJointState();

    // Position should have increased
    EXPECT_GT(final_state.positions[0], initial_state.positions[0]);
}

TEST_F(VirtualControllerTest, GcodeExecution) {
    controller_->connect();
    controller_->enable();
    controller_->home();

    // Send G-code to move joint 1 to 45 degrees
    controller_->sendGcode("G1 A45 F100");

    // Wait for motion to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto state = controller_->getJointState();
    EXPECT_NEAR(state.positions[0], 45.0 * M_PI / 180.0, 0.01);
}

TEST_F(VirtualControllerTest, EmergencyStop) {
    controller_->connect();
    controller_->enable();

    // Start jogging
    controller_->jog(0, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // E-Stop
    controller_->emergencyStop();

    // Should be disabled
    EXPECT_FALSE(controller_->isEnabled());

    // Velocity should be zero
    auto state = controller_->getJointState();
    EXPECT_NEAR(state.velocities[0], 0.0, 0.001);
}

class CollisionDetectorTest : public ::testing::Test {
protected:
    Simulation::CollisionDetector detector_;
};

TEST_F(CollisionDetectorTest, DetectsBoxBoxCollision) {
    // Add two overlapping boxes
    Simulation::CollisionObject box1;
    box1.type = Simulation::CollisionObject::Type::BOX;
    box1.name = "box1";
    box1.dimensions = Eigen::Vector3d(100, 100, 100);
    box1.transform = Eigen::Matrix4d::Identity();

    Simulation::CollisionObject box2;
    box2.type = Simulation::CollisionObject::Type::BOX;
    box2.name = "box2";
    box2.dimensions = Eigen::Vector3d(100, 100, 100);
    box2.transform = Eigen::Matrix4d::Identity();
    box2.transform(0, 3) = 50;  // Translate 50mm in X (overlapping)

    detector_.addObject(box1);
    detector_.addObject(box2);

    // This should detect collision
    // (Implementation depends on how collision is checked)
}

TEST_F(CollisionDetectorTest, NoCollisionWhenSeparated) {
    Simulation::CollisionObject box1;
    box1.type = Simulation::CollisionObject::Type::BOX;
    box1.name = "box1";
    box1.dimensions = Eigen::Vector3d(100, 100, 100);
    box1.transform = Eigen::Matrix4d::Identity();

    Simulation::CollisionObject box2;
    box2.type = Simulation::CollisionObject::Type::BOX;
    box2.name = "box2";
    box2.dimensions = Eigen::Vector3d(100, 100, 100);
    box2.transform = Eigen::Matrix4d::Identity();
    box2.transform(0, 3) = 200;  // Translate 200mm in X (not overlapping)

    detector_.addObject(box1);
    detector_.addObject(box2);

    // This should NOT detect collision
}
```

### 10.2 Integration Test Checklist

```
Simulation Mode Integration Tests:

[ ] VirtualController
    [ ] Connect/Disconnect
    [ ] Enable/Disable
    [ ] Jog all axes
    [ ] G-code execution (G0, G1, G4)
    [ ] Emergency stop
    [ ] Home sequence
    [ ] Time scale adjustment

[ ] Motion Simulation
    [ ] Velocity limits respected
    [ ] Acceleration limits respected
    [ ] Smooth motion profiles
    [ ] Trajectory following accuracy

[ ] Collision Detection
    [ ] Self-collision detection
    [ ] Environment collision
    [ ] Collision visualization
    [ ] Reachability check

[ ] Virtual I/O
    [ ] Digital input simulation
    [ ] Digital output response
    [ ] Analog I/O
    [ ] Linked I/O (output triggers input)

[ ] Welding Simulation
    [ ] Arc on/off
    [ ] Gas flow simulation
    [ ] Wire feed simulation
    [ ] Weld trail visualization

[ ] HMI Integration
    [ ] Mode switching (Real ↔ Sim)
    [ ] Time control panel
    [ ] I/O injection
    [ ] 3D visualization sync
```

---

## 11. Usage Guide

### 11.1 Switching to Simulation Mode

```cpp
// In application startup
void RobotApplication::initialize() {
    // Check if hardware is available
    bool hardware_available = checkHardwareConnection();

    if (hardware_available && !force_simulation_) {
        // Real mode
        controller_ = std::make_unique<GrblHalDriver>(serial_config_);
        spdlog::info("Starting in REAL MODE");
    } else {
        // Simulation mode
        Motion::SimulationConfig sim_config;
        sim_config.update_rate_hz = 1000;
        controller_ = std::make_unique<VirtualController>(sim_config);
        spdlog::info("Starting in SIMULATION MODE");
    }

    // Rest of initialization uses IMotionController interface
    trajectory_planner_.setController(controller_.get());
    state_manager_.setController(controller_.get());
}
```

### 11.2 Program Validation Workflow

```cpp
// Validate program before running on real robot
bool validateProgram(const std::string& program) {
    // Create simulation engine
    Simulation::SimulationEngine sim;
    sim.start();

    // Load program
    auto& controller = sim.getController();
    controller.enable();
    controller.home();

    // Execute in simulation
    std::istringstream stream(program);
    std::string line;
    while (std::getline(stream, line)) {
        controller.sendGcode(line);
    }

    // Wait for completion
    while (/* motion in progress */) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Check for collisions
        if (sim.getMetrics().collisions_detected > 0) {
            spdlog::error("Collision detected in simulation!");
            return false;
        }
    }

    // Report results
    auto metrics = sim.getMetrics();
    spdlog::info("Program validated successfully:");
    spdlog::info("  - Estimated cycle time: {:.2f}s", metrics.simulation_time_s);
    spdlog::info("  - Total distance: {:.1f}mm", metrics.total_distance_mm);
    spdlog::info("  - G-codes executed: {}", metrics.gcode_executed);

    return true;
}
```

---

## 12. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2026-02-01 | System | Initial draft |

---

## 13. References

- CORE_05_Kinematics.md - Forward/Inverse kinematics
- CORE_06_Trajectory.md - Trajectory planning
- CORE_07_grblHAL.md - Real controller interface
- CORE_08_HMI_Framework.md - 3D visualization
- PHASE_2_Motion.md - Simulation Mode requirements
