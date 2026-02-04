# TEST PLAN: PHASE 2 - MOTION CORE

| Metadata      | Value                           |
|---------------|---------------------------------|
| Phase         | 2 - Motion Core                 |
| Version       | 1.0                             |
| Status        | DRAFT                           |
| Last Updated  | 2026-02-01                      |

---

## 1. Test Overview

### 1.1 Scope

Test plan này cover tất cả components trong Phase 2:
- SystemStateManager (FSM)
- Safety Interlocks
- Kinematics Module (IK/FK)
- Trajectory Generator (Ruckig)
- grblHAL Driver
- Jog Mode
- Simulation Mode (VirtualController)

### 1.2 Test Levels

| Level | Description | Tools |
|-------|-------------|-------|
| **Unit Test** | Individual modules | Google Test |
| **Integration Test** | Module interactions | Custom harness |
| **Hardware Test** | Real robot testing | Manual + Automated |
| **Safety Test** | Safety system verification | Manual with witness |

### 1.3 Safety Notice

⚠️ **WARNING**: Phase 2 tests involve robot motion. Follow these safety rules:

1. **Simulation First**: Run all tests in Simulation Mode before hardware
2. **Reduced Speed**: Use 10% speed for initial hardware tests
3. **Clear Area**: Ensure robot work envelope is clear of personnel
4. **E-Stop Ready**: Keep E-Stop within reach at all times
5. **Two-Person Rule**: Hardware tests require operator + safety observer

---

## 2. Unit Tests

### 2.1 State Machine Tests

#### TEST-P2-U001: State Transitions - Normal Flow
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U001 |
| **Component** | SystemStateManager |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Initialize StateManager in BOOT state
2. Trigger initialization complete
3. Verify transition to IDLE
4. Request ARM
5. Verify transition to ARMING
6. Complete arming sequence
7. Verify transition to ARMED
8. Start operation
9. Verify transition to OPERATION

**State Transition Diagram:**
```
BOOT → IDLE → ARMING → ARMED → OPERATION
```

**Test Code:**
```cpp
TEST(StateManagerTest, NormalStateFlow) {
    SystemStateManager sm;

    EXPECT_EQ(sm.getState(), SystemState::BOOT);

    sm.onInitComplete();
    EXPECT_EQ(sm.getState(), SystemState::IDLE);

    EXPECT_TRUE(sm.requestArm());
    EXPECT_EQ(sm.getState(), SystemState::ARMING);

    sm.onArmingComplete();
    EXPECT_EQ(sm.getState(), SystemState::ARMED);

    EXPECT_TRUE(sm.requestOperation());
    EXPECT_EQ(sm.getState(), SystemState::OPERATION);
}
```

---

#### TEST-P2-U002: E-Stop State Transition
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U002 |
| **Component** | SystemStateManager |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Enter OPERATION state
2. Trigger E-Stop
3. Verify immediate transition to ESTOP
4. Verify motion stopped
5. Reset E-Stop
6. Verify transition to IDLE (not OPERATION)

**Test Code:**
```cpp
TEST(StateManagerTest, EStopFromOperation) {
    SystemStateManager sm;
    sm.forceState(SystemState::OPERATION);  // For testing

    // Trigger E-Stop
    sm.onEStop();

    EXPECT_EQ(sm.getState(), SystemState::ESTOP);
    EXPECT_TRUE(sm.isMotionBlocked());

    // Reset E-Stop
    sm.onEStopReset();

    EXPECT_EQ(sm.getState(), SystemState::IDLE);
    EXPECT_FALSE(sm.isMotionBlocked());
}

TEST(StateManagerTest, EStopFromAnyState) {
    std::vector<SystemState> states = {
        SystemState::IDLE,
        SystemState::ARMING,
        SystemState::ARMED,
        SystemState::OPERATION,
        SystemState::STOPPING
    };

    for (auto state : states) {
        SystemStateManager sm;
        sm.forceState(state);

        sm.onEStop();

        EXPECT_EQ(sm.getState(), SystemState::ESTOP)
            << "E-Stop failed from state " << static_cast<int>(state);
    }
}
```

---

#### TEST-P2-U003: Invalid State Transitions
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U003 |
| **Component** | SystemStateManager |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Attempt invalid transitions
2. Verify state does not change
3. Verify error returned

**Test Code:**
```cpp
TEST(StateManagerTest, InvalidTransitions) {
    SystemStateManager sm;
    sm.forceState(SystemState::IDLE);

    // Cannot go directly to OPERATION from IDLE
    EXPECT_FALSE(sm.requestOperation());
    EXPECT_EQ(sm.getState(), SystemState::IDLE);

    // Cannot ARM from ESTOP
    sm.forceState(SystemState::ESTOP);
    EXPECT_FALSE(sm.requestArm());
    EXPECT_EQ(sm.getState(), SystemState::ESTOP);
}
```

---

### 2.2 Kinematics Tests

#### TEST-P2-U010: Forward Kinematics
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U010 |
| **Component** | KinematicsSolver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Set known joint positions
2. Calculate FK
3. Compare with expected TCP position

**Test Data:**
| Joint Config | Expected TCP (X, Y, Z) mm |
|--------------|---------------------------|
| All zeros | (X0, Y0, Z0) - from CAD |
| J1=90° | (Y0, -X0, Z0) rotated |
| J2=90° | Arm extended differently |

**Test Code:**
```cpp
TEST(KinematicsTest, ForwardKinematicsZeroPosition) {
    KinematicsSolver kin;
    kin.loadRobot("models/robot_arm.urdf");

    std::array<double, 6> joints = {0, 0, 0, 0, 0, 0};

    auto tcp = kin.forwardKinematics(joints);

    // Compare with known home position from CAD
    EXPECT_NEAR(tcp.x, 600.0, 0.1);   // mm
    EXPECT_NEAR(tcp.y, 0.0, 0.1);
    EXPECT_NEAR(tcp.z, 800.0, 0.1);
    EXPECT_NEAR(tcp.rx, 0.0, 0.01);   // rad
    EXPECT_NEAR(tcp.ry, 0.0, 0.01);
    EXPECT_NEAR(tcp.rz, 0.0, 0.01);
}

TEST(KinematicsTest, ForwardKinematicsJ1Rotation) {
    KinematicsSolver kin;
    kin.loadRobot("models/robot_arm.urdf");

    std::array<double, 6> joints = {M_PI/2, 0, 0, 0, 0, 0};  // J1 = 90°

    auto tcp = kin.forwardKinematics(joints);

    // X and Y should swap (rotated 90° around Z)
    EXPECT_NEAR(tcp.x, 0.0, 0.1);
    EXPECT_NEAR(tcp.y, 600.0, 0.1);
    EXPECT_NEAR(tcp.z, 800.0, 0.1);
}
```

---

#### TEST-P2-U011: Inverse Kinematics - Valid Solution
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U011 |
| **Component** | KinematicsSolver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Define reachable TCP pose
2. Calculate IK
3. Verify FK of solution matches original pose

**Test Code:**
```cpp
TEST(KinematicsTest, InverseKinematicsRoundTrip) {
    KinematicsSolver kin;
    kin.loadRobot("models/robot_arm.urdf");

    // Start from known joint config
    std::array<double, 6> original_joints = {0.1, -0.5, 0.3, 0.2, -0.1, 0.4};

    // Calculate TCP position
    auto tcp = kin.forwardKinematics(original_joints);

    // Calculate IK to get back to joints
    auto result = kin.inverseKinematics(tcp, original_joints);  // Use original as seed

    ASSERT_TRUE(result.success);

    // Verify solution produces same TCP
    auto verify_tcp = kin.forwardKinematics(result.joints);

    EXPECT_NEAR(verify_tcp.x, tcp.x, 0.01);
    EXPECT_NEAR(verify_tcp.y, tcp.y, 0.01);
    EXPECT_NEAR(verify_tcp.z, tcp.z, 0.01);
    EXPECT_NEAR(verify_tcp.rx, tcp.rx, 0.001);
    EXPECT_NEAR(verify_tcp.ry, tcp.ry, 0.001);
    EXPECT_NEAR(verify_tcp.rz, tcp.rz, 0.001);
}
```

---

#### TEST-P2-U012: Inverse Kinematics - Unreachable Position
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U012 |
| **Component** | KinematicsSolver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Define unreachable TCP pose
2. Calculate IK
3. Verify failure returned

**Test Code:**
```cpp
TEST(KinematicsTest, InverseKinematicsUnreachable) {
    KinematicsSolver kin;
    kin.loadRobot("models/robot_arm.urdf");

    CartesianPose unreachable;
    unreachable.x = 2000.0;  // Way beyond reach
    unreachable.y = 0.0;
    unreachable.z = 0.0;

    std::array<double, 6> seed = {0, 0, 0, 0, 0, 0};
    auto result = kin.inverseKinematics(unreachable, seed);

    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}
```

---

#### TEST-P2-U013: Joint Limits Check
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U013 |
| **Component** | KinematicsSolver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(KinematicsTest, JointLimitsRespected) {
    KinematicsSolver kin;
    kin.loadRobot("models/robot_arm.urdf");

    // Get joint limits
    auto limits = kin.getJointLimits();

    // Check IK solution respects limits
    CartesianPose reachable;
    reachable.x = 500.0;
    reachable.y = 100.0;
    reachable.z = 400.0;

    auto result = kin.inverseKinematics(reachable);

    if (result.success) {
        for (int i = 0; i < 6; i++) {
            EXPECT_GE(result.joints[i], limits[i].min);
            EXPECT_LE(result.joints[i], limits[i].max);
        }
    }
}
```

---

### 2.3 Trajectory Generator Tests

#### TEST-P2-U020: Ruckig OTG - Point to Point
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U020 |
| **Component** | TrajectoryGenerator |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Set start position
2. Set target position
3. Generate trajectory
4. Verify velocity and acceleration limits respected

**Test Code:**
```cpp
TEST(TrajectoryTest, PointToPointRespectLimits) {
    TrajectoryGenerator traj;

    TrajectoryConfig config;
    config.max_velocity = {1.0, 1.0, 1.0, 2.0, 2.0, 2.0};      // rad/s
    config.max_acceleration = {5.0, 5.0, 5.0, 10.0, 10.0, 10.0}; // rad/s²
    config.max_jerk = {50.0, 50.0, 50.0, 100.0, 100.0, 100.0};  // rad/s³
    traj.configure(config);

    std::array<double, 6> start = {0, 0, 0, 0, 0, 0};
    std::array<double, 6> target = {1.0, -0.5, 0.5, 0.2, -0.2, 0.1};

    auto result = traj.planPTP(start, target);

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.duration, 0);

    // Sample trajectory and check limits
    double dt = 0.001;  // 1ms
    for (double t = 0; t <= result.duration; t += dt) {
        auto state = traj.sample(t);

        for (int i = 0; i < 6; i++) {
            EXPECT_LE(std::abs(state.velocity[i]), config.max_velocity[i] * 1.01);
            EXPECT_LE(std::abs(state.acceleration[i]), config.max_acceleration[i] * 1.01);
        }
    }
}
```

---

#### TEST-P2-U021: Trajectory Reaches Target
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U021 |
| **Component** | TrajectoryGenerator |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(TrajectoryTest, ReachesTarget) {
    TrajectoryGenerator traj;
    traj.configure(getDefaultConfig());

    std::array<double, 6> start = {0, 0, 0, 0, 0, 0};
    std::array<double, 6> target = {0.5, -0.3, 0.2, 0.1, -0.1, 0.05};

    auto result = traj.planPTP(start, target);
    ASSERT_TRUE(result.success);

    // Sample at end of trajectory
    auto final_state = traj.sample(result.duration);

    for (int i = 0; i < 6; i++) {
        EXPECT_NEAR(final_state.position[i], target[i], 1e-6);
        EXPECT_NEAR(final_state.velocity[i], 0.0, 1e-6);  // Stopped
    }
}
```

---

### 2.4 Simulation Mode Tests

#### TEST-P2-U030: VirtualController Motion
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U030 |
| **Component** | VirtualController |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(VirtualControllerTest, JogMotion) {
    VirtualController vc;
    vc.connect();
    vc.enable();

    auto initial = vc.getJointState();

    // Jog joint 0 positive
    vc.jog(0, 0.5);  // 0.5 rad/s
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    vc.stopJog();

    auto final_state = vc.getJointState();

    // Should have moved approximately 0.1 rad (0.5 * 0.2s)
    EXPECT_GT(final_state.positions[0], initial.positions[0]);
    EXPECT_NEAR(final_state.positions[0] - initial.positions[0], 0.1, 0.02);
}
```

---

#### TEST-P2-U031: VirtualController E-Stop
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-U031 |
| **Component** | VirtualController |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(VirtualControllerTest, EmergencyStop) {
    VirtualController vc;
    vc.connect();
    vc.enable();

    // Start motion
    vc.jog(0, 1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // E-Stop
    vc.emergencyStop();

    auto state = vc.getJointState();

    // Velocity should be zero
    EXPECT_NEAR(state.velocities[0], 0.0, 0.001);

    // Should be disabled
    EXPECT_FALSE(vc.isEnabled());
}
```

---

## 3. Integration Tests

### 3.1 Motion Pipeline Integration

#### TEST-P2-I001: Jog Command Flow
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-I001 |
| **Component** | UI → Core → VirtualController |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Start system in Simulation Mode
2. Enter OPERATION state
3. Send jog command from UI
4. Verify motion in VirtualController
5. Verify position update in UI

**Test Procedure:**
```
[UI] Click Jog J1+ button
  ↓
[IPC] Send: {"command": "jog", "axis": 0, "velocity": 0.1}
  ↓
[Core] StateManager validates state == OPERATION
  ↓
[Core] TrajectoryGenerator creates jog trajectory
  ↓
[Core] VirtualController receives motion command
  ↓
[VirtualController] Updates joint positions
  ↓
[IPC] Status update: {"positions": [...]}
  ↓
[UI] 3D robot model updates
```

**Pass Criteria:**
- [ ] Jog command accepted within 50ms
- [ ] Robot starts moving within 100ms
- [ ] Position updates at 30Hz to UI
- [ ] Robot stops when button released

---

#### TEST-P2-I002: State Machine - Safety Interlock
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-I002 |
| **Component** | StateManager + SafetyInterlock |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. System in OPERATION state
2. Simulate E-Stop signal
3. Verify motion stops immediately
4. Verify state transitions to ESTOP
5. Verify motion commands rejected

**Test Code:**
```cpp
TEST(MotionIntegration, EStopStopsMotion) {
    // Setup complete motion pipeline
    SystemStateManager state_mgr;
    SafetyInterlock safety(state_mgr);
    VirtualController controller;
    MotionManager motion(state_mgr, controller);

    // Enter OPERATION state
    state_mgr.forceState(SystemState::OPERATION);
    controller.enable();

    // Start continuous motion
    motion.startJog(0, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_TRUE(controller.isMoving());

    // Trigger E-Stop
    safety.triggerEStop();

    // Verify immediate stop
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_FALSE(controller.isMoving());
    EXPECT_EQ(state_mgr.getState(), SystemState::ESTOP);

    // Verify motion commands rejected
    EXPECT_FALSE(motion.startJog(0, 0.5));
}
```

---

### 3.2 Kinematics-Trajectory Integration

#### TEST-P2-I010: Cartesian Jog
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-I010 |
| **Component** | Kinematics + Trajectory + Controller |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Start from known position
2. Jog in X direction (Cartesian)
3. Verify TCP moves in X only
4. Verify joint positions updated via IK

**Test Code:**
```cpp
TEST(CartesianJogTest, JogXDirection) {
    KinematicsSolver kin;
    TrajectoryGenerator traj;
    VirtualController controller;

    // Initialize at home position
    std::array<double, 6> home = {0, 0, 0, 0, 0, 0};
    controller.setJointPositions(home);

    auto initial_tcp = kin.forwardKinematics(home);

    // Jog in +X at 10 mm/s for 100ms
    double jog_velocity = 10.0;  // mm/s
    double jog_time = 0.1;       // seconds

    // ... execute cartesian jog ...

    auto final_joints = controller.getJointState().positions;
    auto final_tcp = kin.forwardKinematics(final_joints);

    // X should have increased by ~1mm
    EXPECT_NEAR(final_tcp.x - initial_tcp.x, jog_velocity * jog_time, 0.5);

    // Y and Z should be unchanged
    EXPECT_NEAR(final_tcp.y, initial_tcp.y, 0.1);
    EXPECT_NEAR(final_tcp.z, initial_tcp.z, 0.1);
}
```

---

## 4. Hardware Tests

### 4.1 grblHAL Communication Tests

#### TEST-P2-H001: Serial Connection
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H001 |
| **Component** | grblHAL Driver |
| **Priority** | P0 |
| **Type** | Manual + Automated |

**Prerequisites:**
- Teensy 4.1 connected via USB
- grblHAL firmware loaded
- Correct COM port identified

**Test Steps:**
1. Open serial connection
2. Send `$I` command
3. Verify firmware version response
4. Close connection

**Test Code:**
```cpp
TEST(GrblHalTest, SerialConnection) {
    GrblHalDriver driver;

    // Find COM port (or use config)
    std::string port = "COM3";  // From config

    ASSERT_TRUE(driver.connect(port, 115200));
    EXPECT_TRUE(driver.isConnected());

    // Query version
    auto response = driver.sendCommand("$I");

    EXPECT_TRUE(response.find("grblHAL") != std::string::npos);

    driver.disconnect();
}
```

**Pass Criteria:**
- [ ] Connection established within 2 seconds
- [ ] Firmware version received
- [ ] No communication errors

---

#### TEST-P2-H002: G-code Command Execution
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H002 |
| **Component** | grblHAL Driver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Connect to grblHAL
2. Send simple G-code commands
3. Verify "ok" response
4. Query position

**Test Code:**
```cpp
TEST(GrblHalTest, GcodeExecution) {
    GrblHalDriver driver;
    driver.connect("COM3", 115200);

    // Send G-code
    auto response = driver.sendCommand("G90");  // Absolute mode
    EXPECT_EQ(response, "ok");

    response = driver.sendCommand("G0 A10 B0 C0 U0 V0 W0");  // Move J1 to 10°
    EXPECT_EQ(response, "ok");

    // Wait for motion complete
    driver.waitForIdle(5000);

    // Query position
    auto pos = driver.getPosition();
    EXPECT_NEAR(pos[0], 10.0, 0.1);  // J1 at 10°
}
```

---

### 4.2 Motion Verification Tests

#### TEST-P2-H010: Single Axis Jog
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H010 |
| **Component** | Complete motion system |
| **Priority** | P0 |
| **Type** | Manual |

**⚠️ SAFETY: Perform in reduced speed mode (10%)**

**Prerequisites:**
- Robot powered on
- E-Stop accessible
- Work area clear
- Observer present

**Test Procedure:**

| Step | Action | Expected Result | Verified |
|------|--------|-----------------|----------|
| 1 | Enable robot from UI | Status shows "ARMED" | [ ] |
| 2 | Enter Operation mode | Status shows "OPERATION" | [ ] |
| 3 | Select J1 in Jog panel | J1 highlighted | [ ] |
| 4 | Press J1+ button | J1 rotates positive | [ ] |
| 5 | Release button | J1 stops immediately | [ ] |
| 6 | Press J1- button | J1 rotates negative | [ ] |
| 7 | Release button | J1 stops immediately | [ ] |
| 8 | Repeat for J2-J6 | All axes move correctly | [ ] |

**Pass Criteria:**
- [ ] All axes move in correct direction
- [ ] Motion stops within 100ms of button release
- [ ] Position display updates in real-time
- [ ] No unusual sounds or vibrations

---

#### TEST-P2-H011: Cartesian Jog
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H011 |
| **Component** | Kinematics + Motion |
| **Priority** | P0 |
| **Type** | Manual |

**Test Procedure:**

| Step | Action | Expected Result | Verified |
|------|--------|-----------------|----------|
| 1 | Switch to Cartesian mode | Mode indicator shows "CART" | [ ] |
| 2 | Press X+ button | TCP moves in +X direction | [ ] |
| 3 | Verify with dial indicator | Movement is in World X | [ ] |
| 4 | Press Y+ button | TCP moves in +Y direction | [ ] |
| 5 | Press Z+ button | TCP moves in +Z (up) | [ ] |
| 6 | Press Rx+ button | TCP rotates around X | [ ] |
| 7 | Press Ry+ button | TCP rotates around Y | [ ] |
| 8 | Press Rz+ button | TCP rotates around Z | [ ] |

**Measurement:**
- Use dial indicator or laser to verify linear motion is correct axis
- Rotation should be around TCP, not base

---

### 4.3 Safety Tests

#### TEST-P2-H020: E-Stop Response Time
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H020 |
| **Component** | Safety System |
| **Priority** | P0 |
| **Type** | Manual with Measurement |

**⚠️ CRITICAL SAFETY TEST - Requires witness signature**

**Test Setup:**
- High-speed camera or oscilloscope
- Robot in motion at test speed

**Test Procedure:**

| Step | Action | Expected Result | Verified |
|------|--------|-----------------|----------|
| 1 | Start robot motion (jog) | Robot moving | [ ] |
| 2 | Record motion start | Timestamp T0 | [ ] |
| 3 | Press E-Stop button | Record timestamp T1 | [ ] |
| 4 | Robot stops completely | Record timestamp T2 | [ ] |
| 5 | Calculate response time | T2 - T1 < 500ms | [ ] |

**Pass Criteria:**
- [ ] E-Stop response time < 500ms (from button press to complete stop)
- [ ] All axes stop (no single axis continues)
- [ ] State transitions to ESTOP
- [ ] Motion commands rejected after E-Stop

**Witness Signature:** _________________ Date: _________

---

#### TEST-P2-H021: Joint Limit Enforcement
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H021 |
| **Component** | Safety Interlock |
| **Priority** | P0 |
| **Type** | Manual |

**Test Procedure:**

| Step | Action | Expected Result | Verified |
|------|--------|-----------------|----------|
| 1 | Jog J1 toward positive limit | Motion normal | [ ] |
| 2 | Approach limit (within 5°) | Speed reduces | [ ] |
| 3 | Reach limit | Motion stops | [ ] |
| 4 | Try to continue past limit | Motion blocked | [ ] |
| 5 | Jog in opposite direction | Motion allowed | [ ] |
| 6 | Repeat for all axes | All limits work | [ ] |

**Pass Criteria:**
- [ ] Robot cannot exceed joint limits
- [ ] Soft limit warning appears before hard limit
- [ ] User can jog away from limit
- [ ] Limit approach reduces speed

---

#### TEST-P2-H022: Deadman Switch
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-H022 |
| **Component** | Safety Interlock |
| **Priority** | P0 |
| **Type** | Manual |

**Test Procedure:**

| Step | Action | Expected Result | Verified |
|------|--------|-----------------|----------|
| 1 | Press deadman switch | Enable button active | [ ] |
| 2 | Start jog motion | Robot moves | [ ] |
| 3 | Release deadman | Robot stops immediately | [ ] |
| 4 | Try motion without deadman | Motion blocked | [ ] |

---

## 5. System Tests

### 5.1 Milestone Verification

#### TEST-P2-S001: Milestone "Robot Moves"
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-S001 |
| **Component** | All Phase 2 |
| **Priority** | P0 |
| **Type** | Manual System Test |

**Milestone Criteria Checklist:**

| # | Criterion | Test Method | Result |
|---|-----------|-------------|--------|
| 1 | State Machine works correctly | TEST-P2-U001-U003 | [ ] |
| 2 | E-Stop stops robot immediately | TEST-P2-H020 | [ ] |
| 3 | Jog Joint mode works | TEST-P2-H010 | [ ] |
| 4 | Jog Cartesian mode works | TEST-P2-H011 | [ ] |
| 5 | Simulation Mode works | TEST-P2-U030-U031 | [ ] |
| 6 | Real Mode works | TEST-P2-H001-H002 | [ ] |

**Simulation Mode Demo:**
- [ ] Robot model updates in 3D view
- [ ] Position values update in UI
- [ ] Jog controls functional
- [ ] No hardware required

**Real Mode Demo:**
- [ ] Robot physically moves
- [ ] Position feedback accurate
- [ ] Smooth motion
- [ ] Correct direction

---

## 6. Performance Tests

### 6.1 Control Loop Timing

#### TEST-P2-P001: Control Loop Cycle Time
| Field | Value |
|-------|-------|
| **ID** | TEST-P2-P001 |
| **Component** | Motion Control Loop |
| **Priority** | P1 |
| **Type** | Automated + Measurement |

**Requirement:** Control loop < 1ms

**Test Procedure:**
1. Instrument control loop with timing
2. Run for 10 seconds
3. Record min/max/avg cycle time

**Test Code:**
```cpp
TEST(PerformanceTest, ControlLoopTiming) {
    MotionController controller;
    controller.start();

    std::vector<double> cycle_times;

    for (int i = 0; i < 10000; i++) {
        auto start = std::chrono::high_resolution_clock::now();

        controller.update();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end - start);
        cycle_times.push_back(duration.count());
    }

    double avg = std::accumulate(cycle_times.begin(), cycle_times.end(), 0.0)
                 / cycle_times.size();
    double max = *std::max_element(cycle_times.begin(), cycle_times.end());
    double min = *std::min_element(cycle_times.begin(), cycle_times.end());

    std::cout << "Control loop timing:" << std::endl;
    std::cout << "  Min: " << min << " ms" << std::endl;
    std::cout << "  Avg: " << avg << " ms" << std::endl;
    std::cout << "  Max: " << max << " ms" << std::endl;

    EXPECT_LT(avg, 1.0);   // Average < 1ms
    EXPECT_LT(max, 5.0);   // Max < 5ms (allow some jitter)
}
```

---

## 7. Test Report Template

```
================================================================================
                         PHASE 2 TEST REPORT
================================================================================

Date: _______________
Tester: _______________
Build Version: _______________
Hardware: _______________

SUMMARY
-------
Unit Tests:        ___ / ___ passed
Integration Tests: ___ / ___ passed
Hardware Tests:    ___ / ___ passed
Safety Tests:      ___ / ___ passed (WITNESS REQUIRED)
Performance Tests: ___ / ___ passed

SAFETY TEST WITNESS
-------------------
I verify that safety tests were performed correctly and results are accurate.

Witness Name: _______________
Signature: _______________
Date: _______________

MILESTONE STATUS
----------------
[ ] MILESTONE "Robot Moves" PASSED
[ ] MILESTONE "Robot Moves" FAILED - See issues below

ISSUES FOUND
------------
1. [BUG-XXX] Description
   - Severity: Critical/High/Medium/Low
   - Status: Open/Fixed

PHASE 2 APPROVAL
----------------
[ ] APPROVED - Ready for Phase 3
[ ] NOT APPROVED - Issues must be fixed

Approver: _______________
Date: _______________
================================================================================
```

---

## 8. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | System | Initial version |
