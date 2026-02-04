# TEST PLAN: PHASE 3 - WELDING MODE

| Metadata      | Value                           |
|---------------|---------------------------------|
| Phase         | 3 - Welding Mode                |
| Version       | 1.0                             |
| Status        | DRAFT                           |
| Last Updated  | 2026-02-01                      |

---

## 1. Test Overview

### 1.1 Scope

Test plan này cover tất cả components trong Phase 3:
- WeldingSequencer FSM
- I/O Interface (Digital/Analog)
- Arc Control (Pre-flow, Ignition, Burnback)
- Weaving Patterns
- Welding Program Execution
- Weld UI Components

### 1.2 Test Levels

| Level | Description | Tools |
|-------|-------------|-------|
| **Unit Test** | WeldingSequencer, WeavingGenerator | Google Test |
| **Integration Test** | Sequencer + I/O + Motion | Custom harness |
| **Simulation Test** | Full cycle with VirtualController | Automated |
| **Hardware Test** | Real welding on test coupons | Manual |

### 1.3 Safety Notice

⚠️ **WELDING SAFETY REQUIREMENTS**:

1. **PPE Required**: Welding helmet, gloves, protective clothing
2. **Ventilation**: Ensure adequate fume extraction
3. **Fire Safety**: Fire extinguisher within reach
4. **Material**: Use appropriate test coupons
5. **Supervision**: Welding engineer must supervise hardware tests
6. **First Tests**: Use simulation mode or cold runs first

---

## 2. Unit Tests

### 2.1 Welding Sequencer Tests

#### TEST-P3-U001: Welding State Machine - Normal Cycle
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U001 |
| **Component** | WeldingSequencer |
| **Priority** | P0 |
| **Type** | Automated |

**Expected State Flow:**
```
IDLE → PREFLOW → IGNITION → STABILIZE → WELDING → CRATER → BURNBACK → POSTFLOW → IDLE
```

**Test Code:**
```cpp
TEST(WeldingSequencerTest, NormalWeldCycle) {
    MockIO io;
    WeldingSequencer seq(io);

    WeldingParams params;
    params.preflow_time_ms = 500;
    params.ignition_time_ms = 200;
    params.stabilize_time_ms = 300;
    params.crater_time_ms = 500;
    params.burnback_time_ms = 100;
    params.postflow_time_ms = 1000;

    // Start weld
    EXPECT_EQ(seq.getState(), WeldState::IDLE);

    seq.startWeld(params);
    EXPECT_EQ(seq.getState(), WeldState::PREFLOW);

    // Simulate time passing
    seq.update(500);  // Preflow complete
    EXPECT_EQ(seq.getState(), WeldState::IGNITION);

    seq.update(200);  // Ignition complete
    EXPECT_EQ(seq.getState(), WeldState::STABILIZE);

    seq.update(300);  // Stabilize complete
    EXPECT_EQ(seq.getState(), WeldState::WELDING);

    // Weld for some time, then stop
    seq.update(5000);
    seq.endWeld();
    EXPECT_EQ(seq.getState(), WeldState::CRATER);

    seq.update(500);  // Crater complete
    EXPECT_EQ(seq.getState(), WeldState::BURNBACK);

    seq.update(100);  // Burnback complete
    EXPECT_EQ(seq.getState(), WeldState::POSTFLOW);

    seq.update(1000);  // Postflow complete
    EXPECT_EQ(seq.getState(), WeldState::IDLE);
}
```

---

#### TEST-P3-U002: Welding I/O Signals
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U002 |
| **Component** | WeldingSequencer + I/O |
| **Priority** | P0 |
| **Type** | Automated |

**Expected I/O at each state:**

| State | Gas | Wire | Arc | Voltage | Current |
|-------|-----|------|-----|---------|---------|
| IDLE | OFF | OFF | OFF | 0 | 0 |
| PREFLOW | ON | OFF | OFF | 0 | 0 |
| IGNITION | ON | ON | CMD | Low | Low |
| STABILIZE | ON | ON | ON | Ramp | Ramp |
| WELDING | ON | ON | ON | Set | Set |
| CRATER | ON | ON | ON | Ramp↓ | Ramp↓ |
| BURNBACK | ON | OFF | ON | High | 0 |
| POSTFLOW | ON | OFF | OFF | 0 | 0 |

**Test Code:**
```cpp
TEST(WeldingSequencerTest, IOSignals) {
    MockIO io;
    WeldingSequencer seq(io);

    WeldingParams params;
    params.voltage = 22.0;
    params.current = 180.0;
    params.wire_speed = 8.0;

    // IDLE state
    EXPECT_FALSE(io.getOutput(IO::GAS_VALVE));
    EXPECT_FALSE(io.getOutput(IO::WIRE_FEED));
    EXPECT_FALSE(io.getOutput(IO::ARC_ON));

    seq.startWeld(params);

    // PREFLOW state
    EXPECT_TRUE(io.getOutput(IO::GAS_VALVE));
    EXPECT_FALSE(io.getOutput(IO::WIRE_FEED));
    EXPECT_FALSE(io.getOutput(IO::ARC_ON));

    seq.update(params.preflow_time_ms);  // -> IGNITION

    // IGNITION state
    EXPECT_TRUE(io.getOutput(IO::GAS_VALVE));
    EXPECT_TRUE(io.getOutput(IO::WIRE_FEED));
    EXPECT_TRUE(io.getOutput(IO::ARC_ON));

    seq.update(params.ignition_time_ms);  // -> STABILIZE
    seq.update(params.stabilize_time_ms); // -> WELDING

    // WELDING state - check analog outputs
    EXPECT_NEAR(io.getAnalogOutput(IO::VOLTAGE_CMD), params.voltage / 10.0, 0.01);
    EXPECT_NEAR(io.getAnalogOutput(IO::CURRENT_CMD), params.current / 50.0, 0.01);
}
```

---

#### TEST-P3-U003: Arc Loss Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U003 |
| **Component** | WeldingSequencer |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeldingSequencerTest, ArcLossDetection) {
    MockIO io;
    WeldingSequencer seq(io);

    WeldingParams params;
    params.arc_loss_timeout_ms = 500;

    seq.startWeld(params);

    // Progress to WELDING state
    seq.update(2000);
    EXPECT_EQ(seq.getState(), WeldState::WELDING);

    // Simulate arc established
    io.setInput(IO::ARC_OK, true);
    seq.update(100);

    // Simulate arc loss
    io.setInput(IO::ARC_OK, false);

    // Wait for timeout
    seq.update(400);
    EXPECT_EQ(seq.getState(), WeldState::WELDING);  // Still in welding

    seq.update(200);  // Total 600ms > timeout
    EXPECT_EQ(seq.getState(), WeldState::ERROR);
    EXPECT_EQ(seq.getError(), WeldError::ARC_LOST);
}
```

---

#### TEST-P3-U004: Emergency Stop During Weld
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U004 |
| **Component** | WeldingSequencer |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeldingSequencerTest, EmergencyStopDuringWeld) {
    MockIO io;
    WeldingSequencer seq(io);

    seq.startWeld(getDefaultParams());
    seq.update(2000);  // In WELDING state

    // E-Stop
    seq.emergencyStop();

    // All outputs should be OFF
    EXPECT_FALSE(io.getOutput(IO::GAS_VALVE));
    EXPECT_FALSE(io.getOutput(IO::WIRE_FEED));
    EXPECT_FALSE(io.getOutput(IO::ARC_ON));
    EXPECT_NEAR(io.getAnalogOutput(IO::VOLTAGE_CMD), 0.0, 0.01);
    EXPECT_NEAR(io.getAnalogOutput(IO::CURRENT_CMD), 0.0, 0.01);

    EXPECT_EQ(seq.getState(), WeldState::ERROR);
}
```

---

### 2.2 Weaving Pattern Tests

#### TEST-P3-U010: Zigzag Weave Pattern
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U010 |
| **Component** | WeavingGenerator |
| **Priority** | P1 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeavingTest, ZigzagPattern) {
    WeavingGenerator weave;

    WeavingConfig config;
    config.pattern = WeavingPattern::ZIGZAG;
    config.width_mm = 5.0;
    config.frequency_hz = 2.0;

    weave.configure(config);
    weave.start();

    // Sample one complete cycle (0.5s at 2Hz)
    std::vector<double> offsets;
    double dt = 0.01;  // 10ms

    for (double t = 0; t < 0.5; t += dt) {
        double offset = weave.getOffset(t);
        offsets.push_back(offset);
    }

    // Find min and max
    double min_offset = *std::min_element(offsets.begin(), offsets.end());
    double max_offset = *std::max_element(offsets.begin(), offsets.end());

    // Width should be ±2.5mm
    EXPECT_NEAR(min_offset, -2.5, 0.1);
    EXPECT_NEAR(max_offset, 2.5, 0.1);

    // Zigzag should have linear segments
    // Check midpoint is at edge, not center
    double quarter_cycle = offsets[offsets.size() / 4];
    EXPECT_GT(std::abs(quarter_cycle), 1.0);  // Not near center at quarter cycle
}
```

---

#### TEST-P3-U011: Sine Weave Pattern
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U011 |
| **Component** | WeavingGenerator |
| **Priority** | P1 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeavingTest, SinePattern) {
    WeavingGenerator weave;

    WeavingConfig config;
    config.pattern = WeavingPattern::SINE;
    config.width_mm = 4.0;
    config.frequency_hz = 3.0;

    weave.configure(config);

    // Check sine wave properties
    double t = 0;
    EXPECT_NEAR(weave.getOffset(t), 0.0, 0.01);  // Start at center

    t = 1.0 / (4 * config.frequency_hz);  // Quarter period
    EXPECT_NEAR(weave.getOffset(t), config.width_mm / 2, 0.01);  // At max

    t = 1.0 / (2 * config.frequency_hz);  // Half period
    EXPECT_NEAR(weave.getOffset(t), 0.0, 0.01);  // Back to center
}
```

---

#### TEST-P3-U012: Weave with Dwell
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U012 |
| **Component** | WeavingGenerator |
| **Priority** | P2 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeavingTest, DwellAtEdges) {
    WeavingGenerator weave;

    WeavingConfig config;
    config.pattern = WeavingPattern::ZIGZAG;
    config.width_mm = 5.0;
    config.frequency_hz = 2.0;
    config.left_dwell_ms = 100;
    config.right_dwell_ms = 100;
    config.center_dwell_ms = 0;

    weave.configure(config);

    // During dwell, offset should remain at edge
    // This requires tracking pattern state
    // Test that dwell extends time at edges
}
```

---

### 2.3 Weld Program Tests

#### TEST-P3-U020: Parse Weld Program
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-U020 |
| **Component** | WeldProgramParser |
| **Priority** | P1 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeldProgramTest, ParseValidProgram) {
    WeldProgramParser parser;

    std::string program = R"(
        WELD_PROGRAM: TestProgram
        MATERIAL: Steel
        THICKNESS: 3.0

        SEGMENT 1:
            START: P1
            END: P2
            SPEED: 10.0
            VOLTAGE: 22.0
            CURRENT: 180.0
            WEAVE: ZIGZAG 5.0 2.0

        SEGMENT 2:
            START: P2
            END: P3
            SPEED: 8.0
            VOLTAGE: 24.0
            CURRENT: 200.0
            WEAVE: NONE
    )";

    auto result = parser.parse(program);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.program.name, "TestProgram");
    EXPECT_EQ(result.program.segments.size(), 2);

    auto& seg1 = result.program.segments[0];
    EXPECT_EQ(seg1.speed, 10.0);
    EXPECT_EQ(seg1.voltage, 22.0);
    EXPECT_EQ(seg1.weave.pattern, WeavingPattern::ZIGZAG);
}
```

---

## 3. Integration Tests

### 3.1 Welding Sequence Integration

#### TEST-P3-I001: Complete Weld Cycle - Simulation
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-I001 |
| **Component** | Full welding pipeline |
| **Priority** | P0 |
| **Type** | Automated (Simulation) |

**Test Steps:**
1. Start system in Simulation Mode
2. Load test weld program
3. Execute weld cycle
4. Verify all states traversed
5. Verify motion executed
6. Verify I/O signals correct

**Test Code:**
```cpp
TEST(WeldingIntegration, CompleteCycleSimulation) {
    SimulationEngine sim;
    WeldingManager welding(sim);

    // Load program
    WeldProgram program;
    program.segments.push_back(createTestSegment(
        Point3d(0, 0, 0),
        Point3d(100, 0, 0),
        10.0,  // speed mm/s
        22.0,  // voltage
        180.0  // current
    ));

    // Execute
    welding.loadProgram(program);
    welding.start();

    // Wait for completion (with timeout)
    auto start = std::chrono::steady_clock::now();
    while (welding.isRunning()) {
        sim.update(0.001);
        welding.update(0.001);

        auto elapsed = std::chrono::steady_clock::now() - start;
        ASSERT_LT(elapsed, std::chrono::seconds(30));
    }

    // Verify completion
    EXPECT_EQ(welding.getState(), WeldState::IDLE);
    EXPECT_FALSE(welding.hasError());

    // Verify motion executed
    auto& io = sim.getIO();
    auto& controller = sim.getController();

    // Robot should be at end position
    auto pos = controller.getJointState();
    auto tcp = kinematics.forwardKinematics(pos.positions);
    EXPECT_NEAR(tcp.x, 100.0, 1.0);
}
```

---

#### TEST-P3-I002: Motion + Welding Synchronization
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-I002 |
| **Component** | Motion + WeldingSequencer |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Start weld at point A
2. Move to point B while welding
3. Verify arc ON during entire motion
4. End weld at point B
5. Verify weld length matches motion length

**Test Code:**
```cpp
TEST(WeldingIntegration, MotionWeldSync) {
    SimulationEngine sim;

    Point3d start(0, 0, 100);
    Point3d end(100, 0, 100);
    double weld_speed = 10.0;  // mm/s

    double expected_weld_time = 100.0 / weld_speed;  // 10 seconds

    // Record arc-on time
    double arc_on_time = 0;
    bool arc_was_on = false;
    auto start_time = std::chrono::steady_clock::now();

    // ... execute welding motion ...

    // Verify arc was on for correct duration
    EXPECT_NEAR(arc_on_time, expected_weld_time, 0.5);
}
```

---

#### TEST-P3-I003: Weaving + Motion Integration
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-I003 |
| **Component** | Motion + WeavingGenerator |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Start linear weld with zigzag weave
2. Record TCP path
3. Verify path shows weave pattern
4. Verify weave perpendicular to travel direction

**Test Code:**
```cpp
TEST(WeldingIntegration, WeavingPath) {
    SimulationEngine sim;

    WeldSegment segment;
    segment.start = Point3d(0, 0, 100);
    segment.end = Point3d(200, 0, 100);  // Move in X direction
    segment.speed = 10.0;
    segment.weave.pattern = WeavingPattern::ZIGZAG;
    segment.weave.width_mm = 5.0;
    segment.weave.frequency_hz = 2.0;

    std::vector<Point3d> path_log;

    // Execute and log path
    // ...

    // Analyze path
    double max_y = 0, min_y = 0;
    for (const auto& p : path_log) {
        max_y = std::max(max_y, p.y);
        min_y = std::min(min_y, p.y);
    }

    // Weave should create Y deviation
    EXPECT_NEAR(max_y, 2.5, 0.5);
    EXPECT_NEAR(min_y, -2.5, 0.5);

    // X should progress linearly
    EXPECT_NEAR(path_log.back().x, 200.0, 1.0);
}
```

---

## 4. Simulation Tests

### 4.1 Virtual Welding Tests

#### TEST-P3-SIM001: Weld Trail Visualization
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-SIM001 |
| **Component** | Simulation + HMI |
| **Priority** | P1 |
| **Type** | Manual Verification |

**Test Steps:**
1. Run weld cycle in simulation
2. Observe 3D viewport
3. Verify weld trail appears behind TCP
4. Verify trail color indicates weld state

**Verification Checklist:**
- [ ] Weld trail appears as 3D tube/ribbon
- [ ] Trail color changes based on weld state
- [ ] Trail follows exact TCP path
- [ ] Trail shows weaving pattern if enabled
- [ ] Trail persists after weld complete
- [ ] Trail can be cleared by user

---

#### TEST-P3-SIM002: Arc Simulation
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-SIM002 |
| **Component** | WeldingSimulator |
| **Priority** | P2 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(WeldingSimulatorTest, ArcSimulation) {
    WeldingSimulator sim;

    WeldingParams params;
    params.voltage = 22.0;
    params.current = 180.0;

    sim.startArc(params);

    // Simulate ignition delay
    sim.update(0.05);
    EXPECT_FALSE(sim.isArcEstablished());

    sim.update(0.15);  // Total 200ms
    EXPECT_TRUE(sim.isArcEstablished());

    // Check simulated values
    auto state = sim.getState();
    EXPECT_NEAR(state.voltage, 22.0, 1.0);
    EXPECT_NEAR(state.current, 180.0, 10.0);

    sim.stopArc();
    EXPECT_FALSE(sim.isArcEstablished());
}
```

---

## 5. Hardware Tests

### 5.1 I/O Verification

#### TEST-P3-H001: Digital Output Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H001 |
| **Component** | I/O Interface |
| **Priority** | P0 |
| **Type** | Manual |

**Prerequisites:**
- I/O module connected
- Multimeter or test lamp available

**Test Procedure:**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Command DO0 ON | LED/Lamp ON | [ ] |
| 2 | Command DO0 OFF | LED/Lamp OFF | [ ] |
| 3 | Command DO1 ON | Gas valve clicks | [ ] |
| 4 | Command DO2 ON | Wire feeder starts | [ ] |
| 5 | Command DO3 ON | Arc enable signal | [ ] |
| 6 | Test all DO (0-15) | All respond | [ ] |

---

#### TEST-P3-H002: Digital Input Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H002 |
| **Component** | I/O Interface |
| **Priority** | P0 |
| **Type** | Manual |

**Test Procedure:**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | No input | UI shows DI0=OFF | [ ] |
| 2 | Short DI0 to 24V | UI shows DI0=ON | [ ] |
| 3 | Remove short | UI shows DI0=OFF | [ ] |
| 4 | Connect arc sensor | Arc OK signal works | [ ] |
| 5 | Connect gas sensor | Gas flow signal works | [ ] |

---

#### TEST-P3-H003: Analog Output Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H003 |
| **Component** | I/O Interface |
| **Priority** | P0 |
| **Type** | Manual with Measurement |

**Equipment:**
- Multimeter with DC voltage measurement

**Test Procedure:**

| Step | Command Value | Expected Output | Measured | Pass |
|------|---------------|-----------------|----------|------|
| 1 | AO0 = 0% | 0.0V ± 0.1V | ___V | [ ] |
| 2 | AO0 = 50% | 5.0V ± 0.1V | ___V | [ ] |
| 3 | AO0 = 100% | 10.0V ± 0.1V | ___V | [ ] |
| 4 | AO1 = 25% | 2.5V ± 0.1V | ___V | [ ] |
| 5 | AO1 = 75% | 7.5V ± 0.1V | ___V | [ ] |

---

### 5.2 Cold Run Tests

#### TEST-P3-H010: Cold Run - No Arc
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H010 |
| **Component** | Motion + I/O |
| **Priority** | P0 |
| **Type** | Manual |

**⚠️ NOTE: Arc is DISABLED for this test**

**Purpose:** Verify motion and timing without actual welding

**Test Procedure:**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Disable arc output | Arc cannot start | [ ] |
| 2 | Load test program | Program loaded | [ ] |
| 3 | Start program | Robot moves to start | [ ] |
| 4 | Observe gas output | Gas valve cycles ON/OFF | [ ] |
| 5 | Observe wire output | Wire feeder cycles | [ ] |
| 6 | Verify timing | Matches programmed times | [ ] |
| 7 | Verify path | Robot follows correct path | [ ] |
| 8 | Program completes | Returns to idle | [ ] |

---

#### TEST-P3-H011: Gas Flow Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H011 |
| **Component** | Gas System |
| **Priority** | P0 |
| **Type** | Manual with Measurement |

**Equipment:**
- Gas flow meter

**Test Procedure:**

| Step | Action | Expected | Measured | Pass |
|------|--------|----------|----------|------|
| 1 | Set flow to 10 L/min | 10 L/min ± 1 | ___L/min | [ ] |
| 2 | Set flow to 15 L/min | 15 L/min ± 1 | ___L/min | [ ] |
| 3 | Set flow to 20 L/min | 20 L/min ± 1 | ___L/min | [ ] |
| 4 | Start weld cycle | Preflow starts | | [ ] |
| 5 | End weld cycle | Postflow completes | | [ ] |

---

### 5.3 Welding Tests

#### TEST-P3-H020: Bead-on-Plate Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H020 |
| **Component** | Complete Welding System |
| **Priority** | P0 |
| **Type** | Manual - REQUIRES WELDING ENGINEER |

**⚠️ FULL PPE REQUIRED**

**Test Material:**
- Mild steel plate 6mm x 100mm x 200mm
- Degreased and clean surface

**Test Parameters:**
| Parameter | Value |
|-----------|-------|
| Wire | ER70S-6, 1.0mm |
| Gas | 80% Ar, 20% CO2 |
| Gas Flow | 15 L/min |
| Voltage | 22V |
| Current | 180A |
| Wire Speed | 8 m/min |
| Travel Speed | 10 mm/s |
| CTWD | 15mm |

**Test Procedure:**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Position robot at start | TCP at start position | [ ] |
| 2 | Set weld parameters | As per table above | [ ] |
| 3 | Start weld program | Arc ignites | [ ] |
| 4 | Observe arc | Stable, no spatter | [ ] |
| 5 | Observe bead | Consistent width | [ ] |
| 6 | Program completes | Clean termination | [ ] |
| 7 | Inspect bead | See quality criteria | [ ] |

**Quality Criteria:**

| Criterion | Requirement | Pass |
|-----------|-------------|------|
| Bead width | 6-8mm, consistent | [ ] |
| Bead height | 2-3mm, consistent | [ ] |
| Spatter | Minimal | [ ] |
| Undercut | None visible | [ ] |
| Porosity | None visible | [ ] |
| Start quality | Clean ignition | [ ] |
| End quality | Filled crater | [ ] |

**Photo Documentation:**
- [ ] Photo of bead (top view)
- [ ] Photo of bead (side view)
- [ ] Photo of start point
- [ ] Photo of end point

---

#### TEST-P3-H021: Weaving Weld Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H021 |
| **Component** | Weaving + Welding |
| **Priority** | P1 |
| **Type** | Manual |

**Test Parameters:**
- Same as TEST-P3-H020 plus:
- Weave Pattern: Zigzag
- Weave Width: 5mm
- Weave Frequency: 2 Hz

**Quality Criteria:**

| Criterion | Requirement | Pass |
|-----------|-------------|------|
| Weave pattern visible | Clear zigzag pattern | [ ] |
| Bead width | 10-12mm (wider than no-weave) | [ ] |
| Edge fusion | Good fusion at edges | [ ] |
| Consistency | Pattern consistent entire length | [ ] |

---

#### TEST-P3-H022: Multi-Pass Weld Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-H022 |
| **Component** | Multi-pass Welding |
| **Priority** | P2 |
| **Type** | Manual |

**Test Material:**
- T-joint: 6mm plates

**Test Procedure:**

| Pass | Description | Verified |
|------|-------------|----------|
| 1 | Root pass | [ ] |
| 2 | Fill pass 1 | [ ] |
| 3 | Fill pass 2 | [ ] |
| 4 | Cap pass | [ ] |

**Quality Criteria:**
- [ ] Inter-pass fusion good
- [ ] No porosity between passes
- [ ] Final profile acceptable

---

## 6. System Tests

### 6.1 Milestone Verification

#### TEST-P3-S001: Milestone "Robot Welds"
| Field | Value |
|-------|-------|
| **ID** | TEST-P3-S001 |
| **Component** | All Phase 3 |
| **Priority** | P0 |
| **Type** | Manual System Test |

**Milestone Criteria Checklist:**

| # | Criterion | Test Reference | Result |
|---|-----------|----------------|--------|
| 1 | Complete weld cycle works | TEST-P3-H020 | [ ] |
| 2 | Preflow → Arc → Weld → Crater → Postflow | TEST-P3-U001 | [ ] |
| 3 | Weaving works | TEST-P3-H021 | [ ] |
| 4 | UI displays weld path | TEST-P3-SIM001 | [ ] |
| 5 | Weld parameters configurable | Manual check | [ ] |
| 6 | Emergency stop works during weld | TEST-P3-U004 | [ ] |

---

## 7. Test Report Template

```
================================================================================
                         PHASE 3 TEST REPORT
================================================================================

Date: _______________
Tester: _______________
Welding Engineer: _______________
Build Version: _______________

SUMMARY
-------
Unit Tests:         ___ / ___ passed
Integration Tests:  ___ / ___ passed
Simulation Tests:   ___ / ___ passed
Hardware Tests:     ___ / ___ passed
Welding Tests:      ___ / ___ passed

WELD TEST RESULTS
-----------------
Bead-on-Plate Test:
  - Material: _______________
  - Parameters: V=___, A=___, WFS=___
  - Quality: PASS / FAIL
  - Notes: _______________

Weaving Test:
  - Pattern: _______________
  - Quality: PASS / FAIL
  - Notes: _______________

PHOTOS ATTACHED
---------------
[ ] Bead-on-plate top view
[ ] Bead-on-plate cross-section
[ ] Weave pattern
[ ] Start/End points

MILESTONE STATUS
----------------
[ ] MILESTONE "Robot Welds" PASSED
[ ] MILESTONE "Robot Welds" FAILED

APPROVALS
---------
Tester: _______________
Welding Engineer: _______________
Date: _______________
================================================================================
```

---

## 8. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | System | Initial version |
