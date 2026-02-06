# Design Document: Hybrid Robot Jogging System

## Document Info

| Field | Value |
|-------|-------|
| **Title** | Hybrid Robot Jogging - Simulation + Real-time Sync |
| **Author** | AI Assistant |
| **Created** | 2026-02-07 |
| **Status** | Draft - Awaiting Review |
| **Dependencies** | IK/FK Engine, Firmware Simulator, grblHAL Interface |

---

## 1. Executive Summary

### 1.1. Problem Statement

Project da import duoc URDF/robot vao viewport voi STL meshes hien thi dung. Buoc tiep theo la cho phep nguoi dung **jog (dieu khien tay) robot trong viewport**, dong thoi dam bao:

1. **Simulation Mode**: Robot di chuyen trong viewport khi chua co hardware
2. **Real-time Sync**: Khi hardware duoc ket noi, viewport phan anh chinh xac vi tri robot thuc

### 1.2. Proposed Solution

Hybrid Jogging Architecture voi 3 layer:

```
+------------------+     +------------------+     +------------------+
|   UI Layer       |     |   Core Layer     |     |  Firmware Layer  |
|   (C# WPF)       | <-> |   (C++ Engine)   | <-> |  (Teensy/Sim)    |
+------------------+     +------------------+     +------------------+
       |                        |                        |
   JogPanel              JogController             grblHAL/Simulator
   Viewport              Kinematics(IK/FK)         Step Generation
   MotionVM              TrajectoryGen(Ruckig)     Encoder Feedback
```

### 1.3. Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Jog Space** | Joint-first, Cartesian sau | Joint jog don gian hon, khong can IK |
| **Simulation** | Firmware Simulator trong Core | Cho phep test full pipeline |
| **State Sync** | PUB-SUB @ 100Hz | Dung ZeroMQ pattern da co |
| **Safety** | Enable button + Soft limits | Theo chuan ISO 10218-1 |

---

## 2. System Architecture

### 2.1. Component Diagram

```
                              HYBRID JOGGING ARCHITECTURE
+-----------------------------------------------------------------------------+
|                                  UI LAYER (C#)                               |
|                                                                              |
|  +------------------+    +------------------+    +------------------+        |
|  |   JogPanel.xaml  |    |  MotionControl   |    |  ViewportService |        |
|  |                  |    |  ViewModel       |    |  (Helix 3D)      |        |
|  |  - Joint Select  |    |                  |    |                  |        |
|  |  - +/- Buttons   |--->|  - JogCommand    |--->|  - UpdateJoints  |        |
|  |  - Speed Slider  |    |  - JogState      |    |  - Animate Links |        |
|  |  - Enable Switch |    |  - Override %    |    |                  |        |
|  +------------------+    +------------------+    +------------------+        |
|           |                      |                       ^                   |
|           |                      | IPC (ZeroMQ)          | Joint Angles      |
|           v                      v                       |                   |
+-----------------------------------------------------------------------------+
                                   |
                          ZeroMQ REQ-REP (Commands)
                          ZeroMQ PUB-SUB (Status @ 100Hz)
                                   |
+-----------------------------------------------------------------------------+
|                                 CORE LAYER (C++)                             |
|                                                                              |
|  +------------------+    +------------------+    +------------------+        |
|  |  IpcServer       |    |  JogController   |    |  Kinematics      |        |
|  |                  |    |                  |    |  Engine          |        |
|  |  - JOG_START     |--->|  - ValidateJog   |--->|  - FK (DH Params)|        |
|  |  - JOG_STOP      |    |  - ApplyLimits   |    |  - IK (Future)   |        |
|  |  - JOG_MOVE      |    |  - CalcTarget    |    |                  |        |
|  +------------------+    +------------------+    +------------------+        |
|           |                      |                       |                   |
|           |                      v                       |                   |
|           |              +------------------+            |                   |
|           |              |  Ruckig OTG      |            |                   |
|           |              |                  |            |                   |
|           |              |  - S-Curve Traj  |<-----------+                   |
|           |              |  - Jerk Limited  |                                |
|           |              +------------------+                                |
|           |                      |                                           |
|           v                      v                                           |
|  +------------------+    +------------------+                                |
|  |  StatusPublisher |    |  FirmwareDriver  |                                |
|  |                  |    |                  |                                |
|  |  - JointAngles   |<---|  - SendGcode     |                                |
|  |  - CartesianPose |    |  - RecvStatus    |                                |
|  |  - JogState      |    |  - SimMode Flag  |                                |
|  +------------------+    +------------------+                                |
|                                  |                                           |
+-----------------------------------------------------------------------------+
                                   |
                     USB Serial (Real) / Internal Queue (Sim)
                                   |
+-----------------------------------------------------------------------------+
|                            FIRMWARE LAYER                                    |
|                                                                              |
|  +----------------------------+    +----------------------------+           |
|  |   grblHAL (Teensy 4.1)     |    |   FirmwareSimulator        |           |
|  |   [REAL HARDWARE]          |    |   [IN-PROCESS MOCK]        |           |
|  |                            |    |                            |           |
|  |  - Parse G-code            |    |  - Parse G-code            |           |
|  |  - Step Generation         |    |  - Update Joint State      |           |
|  |  - Encoder Feedback        |    |  - Simulate Motion Time    |           |
|  |  - Safety I/O              |    |  - Return Status Report    |           |
|  +----------------------------+    +----------------------------+           |
|                                                                              |
+-----------------------------------------------------------------------------+
```

### 2.2. State Flow

```
                              JOG STATE MACHINE

     +--------+     Enable      +--------+     Jog+/-     +--------+
     |  IDLE  |---------------->| READY  |--------------->| JOGGING|
     +--------+                 +--------+                +--------+
         ^                          |                         |
         |                          | Disable                 | Release
         |                          v                         | Button
         |                      +--------+                    |
         +----------------------|  IDLE  |<-------------------+
                                +--------+
                                    |
                                    | E-Stop / Limit Hit
                                    v
                                +--------+
                                | ALARM  |
                                +--------+
```

---

## 3. Detailed Design

### 3.1. Jog Modes

#### 3.1.1. Joint Jog (Priority 1 - First Implementation)

**Concept**: Nguoi dung chon 1 joint (J1-J6), nhan +/- de xoay joint do.

**Data Flow**:
```
User Press +   -->  JOG_MOVE(joint=2, direction=1, speed=50%)
                         |
                         v
                    JogController.calculateTarget()
                         |
                         | target_angle = current_angle + delta
                         v
                    Ruckig.update(current, target)
                         |
                         | smooth trajectory point
                         v
                    FirmwareDriver.sendGcode("G1 J2:45.5 F1000")
                         |
                         v
                    grblHAL/Simulator executes
                         |
                         v
                    StatusReport: "J2:45.5"
                         |
                         v
                    Viewport.updateJoint(2, 45.5)
```

**Velocity Profile** (Ruckig S-Curve):
```
Velocity
    ^
    |       ___________
    |      /           \
    |     /             \
    |    /               \
    |___/                 \___
    +-------------------------> Time
       Accel   Cruise   Decel
```

#### 3.1.2. Cartesian Jog (Priority 2 - Requires IK)

**Concept**: Nguoi dung chon axis (X/Y/Z/Rx/Ry/Rz), robot di chuyen theo huong do.

**Data Flow** (khi co IK):
```
User Press X+  -->  JOG_MOVE(axis=X, direction=1, speed=50%)
                         |
                         v
                    Kinematics.FK(current_joints) --> current_pose
                         |
                         v
                    target_pose = current_pose + delta_X
                         |
                         v
                    Ruckig.update(current_pose, target_pose) [Cartesian OTG]
                         |
                         v
                    Kinematics.IK(next_pose) --> next_joints
                         |
                         v
                    FirmwareDriver.sendGcode("G1 J0:x J1:y ...")
```

**Note**: Cartesian jog can IK solver. Se implement o phase sau.

### 3.2. Firmware Simulator

**Purpose**: Cho phep test full jog pipeline khi chua co Teensy hardware.

**Implementation**:

```cpp
// FirmwareSimulator.hpp
class FirmwareSimulator {
public:
    // Parse G-code command and update internal state
    void processCommand(const std::string& gcode);

    // Get current status (giong format grblHAL)
    std::string getStatusReport() const;

    // Simulate motion with realistic timing
    void update(double dt_seconds);

private:
    std::array<double, 6> m_jointPositions;  // Current position
    std::array<double, 6> m_jointTargets;    // Target from G-code
    std::array<double, 6> m_jointVelocities; // Current velocity
    double m_feedRate;                        // From F parameter

    // Motion state
    bool m_isMoving;
    double m_motionProgress;  // 0.0 to 1.0
};
```

**Key Behaviors**:
1. Parse G-code: `G1 J0:45.0 J1:-30.0 F3600`
2. Interpolate joints toward target at feedrate
3. Return status: `<Idle|MPos:45.00,-30.00,0.00,0.00,0.00,0.00|FS:3600,0>`
4. Khi motion xong, return `ok`

### 3.3. IPC Messages

#### 3.3.1. New Message Types

```cpp
// MessageTypes.hpp - Add these
enum class MessageType : uint16_t {
    // ... existing types ...

    // Jog Control (0x0100 - 0x01FF)
    JOG_START     = 0x0100,  // Enable jog mode
    JOG_STOP      = 0x0101,  // Disable jog mode
    JOG_MOVE      = 0x0102,  // Continuous jog command
    JOG_STEP      = 0x0103,  // Incremental jog command
    JOG_STATE     = 0x0104,  // Response: current jog state
};
```

#### 3.3.2. Payload Structures

```cpp
// JogPayloads.hpp

struct JogStartRequest {
    bool enableDeadman;  // Require continuous button hold

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStartRequest, enableDeadman)
};

struct JogMoveRequest {
    int mode;           // 0=Joint, 1=Cartesian
    int axis;           // Joint: 0-5, Cartesian: 0=X,1=Y,2=Z,3=Rx,4=Ry,5=Rz
    int direction;      // -1 or +1
    double speedPercent; // 1-100%

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogMoveRequest, mode, axis, direction, speedPercent)
};

struct JogStepRequest {
    int mode;
    int axis;
    int direction;
    double increment;   // Degrees for joint, mm for Cartesian
    double speedPercent;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStepRequest, mode, axis, direction, increment, speedPercent)
};

struct JogStateResponse {
    bool enabled;
    bool isJogging;
    int currentAxis;
    int currentDirection;
    std::array<double, 6> jointAngles;
    std::array<double, 6> cartesianPose; // X,Y,Z,Rx,Ry,Rz

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStateResponse, enabled, isJogging,
        currentAxis, currentDirection, jointAngles, cartesianPose)
};
```

### 3.4. Viewport Integration

#### 3.4.1. RobotModel3D Updates

Hien tai `RobotModel3D.cs` da co method `UpdateJoints()` de cap nhat goc cac joints. Can mo rong:

```csharp
// RobotModel3D.cs - Add/modify

public class RobotModel3D {
    // ... existing code ...

    /// <summary>
    /// Update joint angles from status feedback
    /// </summary>
    /// <param name="jointAngles">Array of 6 joint angles in degrees</param>
    public void UpdateJointsFromFeedback(double[] jointAngles)
    {
        if (jointAngles.Length != _links.Count - 1) return;

        for (int i = 0; i < jointAngles.Length; i++)
        {
            _jointAngles[i] = jointAngles[i];
        }

        // Recalculate all link transforms using DH parameters
        RecalculateLinkTransforms();

        // Notify viewport to re-render
        OnJointsUpdated?.Invoke(this, EventArgs.Empty);
    }

    public event EventHandler? OnJointsUpdated;
}
```

#### 3.4.2. MainViewModel Subscription

```csharp
// MainViewModel.cs - Subscribe to status updates

private void OnRobotStatusReceived(RobotStatus status)
{
    // Throttle to 30Hz for UI
    if (_lastViewportUpdate.ElapsedMilliseconds < 33) return;

    // Update 3D viewport
    _robotModel.UpdateJointsFromFeedback(status.JointAngles);

    _lastViewportUpdate.Restart();
}
```

---

## 4. Safety Considerations

### 4.1. Jog Safety Requirements (ISO 10218-1)

| Requirement | Implementation |
|-------------|----------------|
| **Enable Device** | Button phai duoc giu lien tuc (Deadman) |
| **Speed Limit T1** | Max 250mm/s trong manual mode |
| **Soft Limits** | Check joint limits truoc khi gui command |
| **E-Stop** | Nhan E-Stop => ngat ngay, transition to ALARM |
| **Two-Hand Control** | Optional: Can 2 tay de jog (cho mode nguy hiem) |

### 4.2. Software Safety Layers

```
+-----------------------------------------+
|  LAYER 4: UI Safety                     |
|  - Enable button visual feedback        |
|  - Disable jog when window loses focus  |
|  - Confirm dialogs for dangerous ops    |
+-----------------------------------------+
          |
+-----------------------------------------+
|  LAYER 3: Core Safety                   |
|  - Soft limit checking                  |
|  - Velocity monitoring                  |
|  - Heartbeat watchdog (150ms timeout)   |
+-----------------------------------------+
          |
+-----------------------------------------+
|  LAYER 2: Firmware Safety               |
|  - Hard limit switches (NC)             |
|  - Watchdog timer (reset if no commands)|
|  - E-Stop circuit                       |
+-----------------------------------------+
          |
+-----------------------------------------+
|  LAYER 1: Hardware Safety               |
|  - Safety relay (STO)                   |
|  - Mechanical brakes                    |
+-----------------------------------------+
```

### 4.3. Deadman Implementation

```csharp
// JogPanel.xaml.cs

private async void JogPlus_MouseDown(object sender, MouseButtonEventArgs e)
{
    if (!ViewModel.IsEnabled) return;

    // Start jog
    await ViewModel.StartJogAsync(direction: 1);

    // Capture mouse to detect release even outside button
    Mouse.Capture((UIElement)sender);
}

private async void Jog_MouseUp(object sender, MouseButtonEventArgs e)
{
    Mouse.Capture(null);

    // Stop jog immediately
    await ViewModel.StopJogAsync();
}

// Also handle window deactivation
private void Window_Deactivated(object sender, EventArgs e)
{
    // Force stop jog if window loses focus
    ViewModel.ForceStopJog();
}
```

---

## 5. Implementation Plan

### 5.1. Dependencies

```
+-------------------+     +-------------------+     +-------------------+
|  PHASE 1          |     |  PHASE 2          |     |  PHASE 3          |
|  (No Dependencies)|     |  (Requires Ph1)   |     |  (Requires Ph2)   |
+-------------------+     +-------------------+     +-------------------+
|                   |     |                   |     |                   |
| - Firmware Sim    |---->| - Joint Jog       |---->| - Cartesian Jog   |
| - Jog Payloads    |     | - Ruckig OTG      |     | - IK Integration  |
| - IPC Handlers    |     | - Viewport Sync   |     | - Seam Tracking   |
|                   |     | - Safety Checks   |     |                   |
+-------------------+     +-------------------+     +-------------------+
```

### 5.2. Phase 1: Foundation (No IK Required)

**Goal**: Co the jog 1 joint trong simulation mode

**Tasks**:
1. [ ] Create `FirmwareSimulator` class (mock grblHAL)
2. [ ] Add jog message types to IPC
3. [ ] Create `JogController` in Core
4. [ ] Wire JogPanel events to IPC commands
5. [ ] Subscribe ViewportService to status updates

**Estimated Effort**: 3-5 days

### 5.3. Phase 2: Full Joint Jog

**Goal**: Jog tat ca 6 joints voi S-curve trajectory

**Tasks**:
1. [ ] Integrate Ruckig for jog trajectory
2. [ ] Implement soft limit checking
3. [ ] Add deadman switch logic
4. [ ] Add jog speed override
5. [ ] Test with all 6 joints

**Estimated Effort**: 3-5 days

### 5.4. Phase 3: Cartesian Jog (Future)

**Goal**: Jog theo X/Y/Z/Rx/Ry/Rz

**Prerequisites**:
- IK solver implemented (Robotics Library or custom)
- FK verified working

**Tasks**:
1. [ ] Implement IK solver
2. [ ] Add Cartesian jog mode
3. [ ] Handle singularity detection
4. [ ] Add coordinate system selection (Base/Tool/World)

**Estimated Effort**: 5-10 days

### 5.5. Phase 4: Hardware Integration (When Teensy Ready)

**Goal**: Sync voi robot thuc

**Tasks**:
1. [ ] Detect Teensy connection (USB enumeration)
2. [ ] Switch between Simulator and Real firmware
3. [ ] Verify encoder feedback loop
4. [ ] Calibrate timing and latency
5. [ ] Full integration test

**Estimated Effort**: 3-5 days

---

## 6. API Reference

### 6.1. C# Client API

```csharp
// IIpcClientService.cs - Add methods

public interface IIpcClientService
{
    // ... existing methods ...

    // Jog Control
    Task<bool> StartJogModeAsync();
    Task<bool> StopJogModeAsync();
    Task<bool> JogMoveAsync(int mode, int axis, int direction, double speedPercent);
    Task<bool> JogStepAsync(int mode, int axis, int direction, double increment, double speedPercent);
}
```

### 6.2. C++ Core API

```cpp
// JogController.hpp

class JogController {
public:
    // Enable/Disable jog mode
    bool enable();
    bool disable();
    bool isEnabled() const;

    // Continuous jog (call repeatedly while button held)
    bool startJog(JogMode mode, int axis, int direction, double speedPercent);
    bool stopJog();

    // Incremental jog (single step)
    bool jogStep(JogMode mode, int axis, int direction, double increment, double speedPercent);

    // Get current state
    JogState getState() const;

private:
    JogState m_state;
    std::unique_ptr<ruckig::Ruckig<6>> m_trajectory;
    // ... kinematics, firmware driver references
};
```

---

## 7. Testing Strategy

### 7.1. Unit Tests

| Test Case | Description |
|-----------|-------------|
| `test_firmware_sim_gcode_parse` | Verify G-code parsing |
| `test_firmware_sim_motion` | Verify motion interpolation |
| `test_jog_limits` | Verify soft limit enforcement |
| `test_jog_deadman` | Verify stop on button release |
| `test_ruckig_scurve` | Verify jerk-limited trajectory |

### 7.2. Integration Tests

| Test Case | Description |
|-----------|-------------|
| `test_jog_e2e_joint` | Full pipeline: UI -> IPC -> Core -> Sim -> IPC -> Viewport |
| `test_jog_estop` | Verify E-Stop halts jog immediately |
| `test_viewport_sync` | Verify 3D model matches simulated position |

### 7.3. Manual Test Checklist

- [ ] Click J1, press +, robot rotates in viewport
- [ ] Release button, robot stops smoothly (S-curve decel)
- [ ] Press J1 to limit, robot stops at soft limit
- [ ] Lose window focus, jog stops automatically
- [ ] Change speed slider, jog speed changes accordingly

---

## 8. Open Questions

1. **IK Solver Choice**: Dung Robotics Library hay implement custom DH-based solver?
2. **Ruckig Licensing**: Community version du cho project hay can Pro?
3. **Coordinate Systems**: Bao nhieu coordinate frames can ho tro? (Base, Tool, World, User-defined)
4. **Jog Increments**: Can ho tro bao nhieu increment presets? (0.1, 1, 10, 90 degrees)

---

## 9. References

1. [Architecture Overview](../01_ARCHITECTURE_OVERVIEW.md)
2. [HMI Research - KUKA WPF](../../ressearch_doc_md/Thiết%20kế%20HMI%20Robot%20KUKA%20WPF.md)
3. [Ruckig Integration](../../ressearch_doc_md/Tích%20hợp%20Ruckig%20cho%20Robot%20Hàn%206-DOF.md)
4. [grblHAL Optimization](../../ressearch_doc_md/Tối%20ưu%20grblHAL%20cho%20Robot%206-DOF.md)
5. ISO 10218-1:2011 - Safety requirements for industrial robots

---

*Document version: 1.0 | Status: Draft*
