# TEST PLAN: PHASE 4 - VISION INTEGRATION

| Metadata      | Value                           |
|---------------|---------------------------------|
| Phase         | 4 - Vision Integration          |
| Version       | 1.0                             |
| Status        | DRAFT                           |
| Last Updated  | 2026-02-01                      |

---

## 1. Test Overview

### 1.1 Scope

Test plan này cover tất cả components trong Phase 4:
- Laser Profiler Driver
- Profile Processing
- Seam Detection Algorithms
- Hand-Eye Calibration
- Real-time Seam Tracking
- Vision UI Components

### 1.2 Test Levels

| Level | Description | Tools |
|-------|-------------|-------|
| **Unit Test** | Individual algorithms | Google Test, OpenCV |
| **Integration Test** | Sensor + Processing | Custom harness |
| **Calibration Test** | Hand-Eye accuracy | Calibration target |
| **Tracking Test** | Real-time performance | Test workpieces |

### 1.3 Equipment Required

| Equipment | Specification | Purpose |
|-----------|---------------|---------|
| Laser Profiler | Keyence LJ-X8000 or equivalent | 3D scanning |
| Calibration Target | Checkerboard 9x7, 20mm squares | Hand-Eye calibration |
| Test Workpieces | V-groove, fillet, butt joint samples | Seam detection testing |
| Dial Indicator | 0.01mm resolution | Position accuracy |

---

## 2. Unit Tests

### 2.1 Laser Profiler Driver Tests

#### TEST-P4-U001: Profiler Connection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U001 |
| **Component** | LaserProfilerDriver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(ProfilerDriverTest, Connection) {
    LaserProfilerDriver driver;

    // Connect to profiler
    ASSERT_TRUE(driver.connect("192.168.1.100", 24691));
    EXPECT_TRUE(driver.isConnected());

    // Get device info
    auto info = driver.getDeviceInfo();
    EXPECT_FALSE(info.model.empty());
    EXPECT_GT(info.resolution_x, 0);

    driver.disconnect();
    EXPECT_FALSE(driver.isConnected());
}
```

---

#### TEST-P4-U002: Profile Acquisition
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U002 |
| **Component** | LaserProfilerDriver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(ProfilerDriverTest, AcquireProfile) {
    LaserProfilerDriver driver;
    driver.connect("192.168.1.100", 24691);

    // Configure acquisition
    ProfileConfig config;
    config.exposure_us = 100;
    config.trigger_mode = TriggerMode::SOFTWARE;
    driver.configure(config);

    // Acquire single profile
    auto profile = driver.acquireProfile();

    ASSERT_TRUE(profile.valid);
    EXPECT_GT(profile.points.size(), 100);  // Expect many points

    // Check point data
    for (const auto& pt : profile.points) {
        // X should be within sensor range
        EXPECT_GE(pt.x, -50.0);  // mm
        EXPECT_LE(pt.x, 50.0);

        // Z (height) should be reasonable
        EXPECT_GE(pt.z, 0.0);
        EXPECT_LE(pt.z, 100.0);

        // Intensity should be valid
        EXPECT_GE(pt.intensity, 0.0);
        EXPECT_LE(pt.intensity, 1.0);
    }
}
```

---

#### TEST-P4-U003: Continuous Streaming
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U003 |
| **Component** | LaserProfilerDriver |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(ProfilerDriverTest, ContinuousStreaming) {
    LaserProfilerDriver driver;
    driver.connect("192.168.1.100", 24691);

    int frame_count = 0;
    driver.setProfileCallback([&](const Profile& profile) {
        frame_count++;
        EXPECT_TRUE(profile.valid);
    });

    // Start streaming at 1kHz
    driver.startStreaming(1000);

    // Wait 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    driver.stopStreaming();

    // Should have received ~1000 frames
    EXPECT_GT(frame_count, 900);
    EXPECT_LT(frame_count, 1100);
}
```

---

### 2.2 Profile Processing Tests

#### TEST-P4-U010: Profile Filtering
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U010 |
| **Component** | ProfileProcessor |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(ProfileProcessorTest, NoiseFiltering) {
    ProfileProcessor processor;

    // Create noisy profile
    Profile noisy;
    for (int i = 0; i < 100; i++) {
        ProfilePoint pt;
        pt.x = i * 1.0;
        pt.z = 10.0 + (rand() % 100) * 0.01;  // 10mm ± noise
        noisy.points.push_back(pt);
    }

    // Add outliers
    noisy.points[50].z = 50.0;  // Spike

    auto filtered = processor.filter(noisy, FilterType::MEDIAN, 5);

    // Outlier should be removed
    EXPECT_LT(filtered.points[50].z, 15.0);

    // Average should be close to 10
    double avg = 0;
    for (const auto& pt : filtered.points) {
        avg += pt.z;
    }
    avg /= filtered.points.size();
    EXPECT_NEAR(avg, 10.0, 0.5);
}
```

---

#### TEST-P4-U011: Edge Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U011 |
| **Component** | ProfileProcessor |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(ProfileProcessorTest, EdgeDetection) {
    ProfileProcessor processor;

    // Create step profile (simulates edge)
    Profile step;
    for (int i = 0; i < 100; i++) {
        ProfilePoint pt;
        pt.x = i * 1.0;
        pt.z = (i < 50) ? 0.0 : 5.0;  // Step at x=50
        step.points.push_back(pt);
    }

    auto edges = processor.detectEdges(step, 0.5);  // 0.5mm/mm threshold

    ASSERT_EQ(edges.size(), 1);
    EXPECT_NEAR(edges[0].x, 50.0, 2.0);
    EXPECT_EQ(edges[0].type, EdgeType::RISING);
}
```

---

### 2.3 Seam Detection Tests

#### TEST-P4-U020: V-Groove Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U020 |
| **Component** | SeamDetector |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(SeamDetectorTest, VGrooveDetection) {
    SeamDetector detector;

    // Create V-groove profile
    Profile vgroove = createVGrooveProfile(
        60.0,   // groove angle (degrees)
        5.0,    // groove depth (mm)
        0.0     // center position (mm)
    );

    auto result = detector.detectSeam(vgroove, SeamType::V_GROOVE);

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.type, SeamType::V_GROOVE);
    EXPECT_NEAR(result.center_x, 0.0, 0.5);  // Center within 0.5mm
    EXPECT_NEAR(result.depth, 5.0, 0.5);     // Depth within 0.5mm
    EXPECT_NEAR(result.angle, 60.0, 2.0);    // Angle within 2°
}
```

---

#### TEST-P4-U021: Fillet Joint Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U021 |
| **Component** | SeamDetector |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(SeamDetectorTest, FilletJointDetection) {
    SeamDetector detector;

    // Create fillet joint profile (90° corner)
    Profile fillet = createFilletProfile(
        90.0,   // angle
        0.0,    // corner position X
        0.0     // corner position Z
    );

    auto result = detector.detectSeam(fillet, SeamType::FILLET);

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.type, SeamType::FILLET);
    EXPECT_NEAR(result.corner_x, 0.0, 0.5);
    EXPECT_NEAR(result.corner_z, 0.0, 0.5);
    EXPECT_NEAR(result.angle, 90.0, 2.0);
}
```

---

#### TEST-P4-U022: Butt Joint Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U022 |
| **Component** | SeamDetector |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(SeamDetectorTest, ButtJointDetection) {
    SeamDetector detector;

    // Create butt joint profile (gap between plates)
    Profile butt = createButtJointProfile(
        2.0,    // gap width (mm)
        0.0,    // gap center X
        6.0     // plate thickness
    );

    auto result = detector.detectSeam(butt, SeamType::BUTT);

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.type, SeamType::BUTT);
    EXPECT_NEAR(result.gap_width, 2.0, 0.3);
    EXPECT_NEAR(result.center_x, 0.0, 0.5);
}
```

---

#### TEST-P4-U023: No Seam Detection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U023 |
| **Component** | SeamDetector |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(SeamDetectorTest, NoSeamDetection) {
    SeamDetector detector;

    // Create flat profile (no seam)
    Profile flat;
    for (int i = 0; i < 100; i++) {
        ProfilePoint pt;
        pt.x = i * 1.0 - 50.0;
        pt.z = 10.0;  // Flat surface
        flat.points.push_back(pt);
    }

    auto result = detector.detectSeam(flat, SeamType::ANY);

    EXPECT_FALSE(result.found);
}
```

---

### 2.4 Hand-Eye Calibration Tests

#### TEST-P4-U030: Calibration Data Collection
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U030 |
| **Component** | HandEyeCalibrator |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(HandEyeCalibTest, DataCollection) {
    HandEyeCalibrator calibrator;

    // Simulate collecting calibration poses
    for (int i = 0; i < 15; i++) {
        CalibrationPose pose;
        pose.robot_pose = generateRandomPose();
        pose.target_pose = simulateTargetDetection(pose.robot_pose);

        calibrator.addPose(pose);
    }

    EXPECT_EQ(calibrator.getPoseCount(), 15);
    EXPECT_TRUE(calibrator.hasEnoughPoses());
}
```

---

#### TEST-P4-U031: Hand-Eye Matrix Computation
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-U031 |
| **Component** | HandEyeCalibrator |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(HandEyeCalibTest, MatrixComputation) {
    HandEyeCalibrator calibrator;

    // Create synthetic calibration data with known transform
    Eigen::Matrix4d true_hand_eye = createKnownTransform(
        50.0, 10.0, 80.0,  // Translation (mm)
        5.0, -3.0, 2.0     // Rotation (degrees)
    );

    // Generate calibration poses using known transform
    auto poses = generateCalibrationPoses(true_hand_eye, 20);
    for (const auto& pose : poses) {
        calibrator.addPose(pose);
    }

    // Compute calibration
    ASSERT_TRUE(calibrator.compute());

    auto computed = calibrator.getHandEyeMatrix();

    // Compare with ground truth
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_NEAR(computed(i, j), true_hand_eye(i, j), 0.01);
        }
    }

    // Check reprojection error
    EXPECT_LT(calibrator.getReprojectionError(), 0.5);  // < 0.5mm
}
```

---

## 3. Integration Tests

### 3.1 Sensor-Robot Integration

#### TEST-P4-I001: Profile in Robot Frame
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-I001 |
| **Component** | Sensor + Kinematics |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Acquire profile from sensor
2. Transform to robot base frame
3. Verify transformation correct

**Test Code:**
```cpp
TEST(VisionIntegration, ProfileInRobotFrame) {
    LaserProfilerDriver profiler;
    KinematicsSolver kinematics;
    Eigen::Matrix4d T_hand_eye;  // From calibration

    profiler.connect("192.168.1.100", 24691);

    // Get current robot pose
    auto joints = robot.getJointPositions();
    auto T_base_flange = kinematics.forwardKinematicsMatrix(joints);

    // Acquire profile
    auto profile = profiler.acquireProfile();

    // Transform profile points to robot frame
    Eigen::Matrix4d T_base_sensor = T_base_flange * T_hand_eye;

    for (auto& pt : profile.points) {
        Eigen::Vector4d p_sensor(pt.x, 0, pt.z, 1);  // 2D profile in sensor frame
        Eigen::Vector4d p_base = T_base_sensor * p_sensor;

        pt.x = p_base(0);
        pt.y = p_base(1);
        pt.z = p_base(2);
    }

    // Points should now be in robot base frame
    // Verify by checking they're in expected workspace
    for (const auto& pt : profile.points) {
        EXPECT_GT(pt.x, -1000);  // Within workspace
        EXPECT_LT(pt.x, 1000);
        EXPECT_GT(pt.z, 0);      // Above table
    }
}
```

---

### 3.2 Seam Tracking Integration

#### TEST-P4-I010: Real-time Seam Tracking Loop
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-I010 |
| **Component** | Complete tracking pipeline |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Start seam tracking
2. Measure loop rate
3. Verify latency acceptable

**Test Code:**
```cpp
TEST(SeamTrackingIntegration, TrackingLoopPerformance) {
    SeamTracker tracker;
    tracker.initialize();

    std::vector<double> loop_times;

    for (int i = 0; i < 1000; i++) {
        auto start = std::chrono::high_resolution_clock::now();

        // Full tracking cycle
        auto profile = tracker.acquireProfile();
        auto seam = tracker.detectSeam(profile);
        auto correction = tracker.computeCorrection(seam);

        auto end = std::chrono::high_resolution_clock::now();
        double loop_ms = std::chrono::duration<double, std::milli>(end - start).count();
        loop_times.push_back(loop_ms);
    }

    double avg = std::accumulate(loop_times.begin(), loop_times.end(), 0.0)
                 / loop_times.size();
    double max = *std::max_element(loop_times.begin(), loop_times.end());

    std::cout << "Tracking loop: avg=" << avg << "ms, max=" << max << "ms" << std::endl;

    // Requirements
    EXPECT_LT(avg, 5.0);   // Average < 5ms (200Hz)
    EXPECT_LT(max, 20.0);  // Max < 20ms
}
```

---

#### TEST-P4-I011: Correction Application
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-I011 |
| **Component** | Tracking + Motion |
| **Priority** | P0 |
| **Type** | Automated |

**Test Code:**
```cpp
TEST(SeamTrackingIntegration, CorrectionApplication) {
    SeamTracker tracker;
    MotionController motion;

    // Simulate seam deviation
    SeamDetectionResult seam;
    seam.found = true;
    seam.center_x = 2.5;  // Seam is 2.5mm to the right

    // Compute correction
    auto correction = tracker.computeCorrection(seam);

    EXPECT_NEAR(correction.lateral_mm, -2.5, 0.1);  // Move left to compensate

    // Apply to motion
    auto current_pose = motion.getCurrentPose();
    auto corrected_pose = tracker.applyCorrection(current_pose, correction);

    // Verify correction applied
    EXPECT_NE(corrected_pose.y, current_pose.y);
}
```

---

## 4. Calibration Tests

### 4.1 Hand-Eye Calibration Procedure

#### TEST-P4-C001: Hand-Eye Calibration Accuracy
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-C001 |
| **Component** | HandEyeCalibrator |
| **Priority** | P0 |
| **Type** | Manual with Measurement |

**Equipment:**
- Calibration target (checkerboard)
- Dial indicator

**Procedure:**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Mount calibration target on table | Target visible to sensor | [ ] |
| 2 | Move robot to 15 different poses | Target detected in all poses | [ ] |
| 3 | Capture pose pairs (robot + target) | 15 pairs collected | [ ] |
| 4 | Run calibration algorithm | Calibration completes | [ ] |
| 5 | Check reprojection error | < 0.5mm | [ ] |

**Validation:**

| Step | Action | Expected | Measured | Pass |
|------|--------|----------|----------|------|
| 1 | Touch known point with TCP | Position recorded | | [ ] |
| 2 | Scan same point with sensor | Point detected | | [ ] |
| 3 | Transform sensor point to base | Match TCP position | | [ ] |
| 4 | Measure error | < 1.0mm | ___mm | [ ] |

---

#### TEST-P4-C002: Calibration Repeatability
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-C002 |
| **Component** | HandEyeCalibrator |
| **Priority** | P1 |
| **Type** | Manual |

**Procedure:**
1. Perform calibration 3 times
2. Compare results

**Results:**

| Run | X (mm) | Y (mm) | Z (mm) | Rx (°) | Ry (°) | Rz (°) |
|-----|--------|--------|--------|--------|--------|--------|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| Std Dev | | | | | | |

**Pass Criteria:**
- [ ] Translation std dev < 0.5mm
- [ ] Rotation std dev < 0.5°

---

## 5. Hardware Tests

### 5.1 Sensor Hardware Tests

#### TEST-P4-H001: Sensor Mounting Verification
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H001 |
| **Component** | Physical setup |
| **Priority** | P0 |
| **Type** | Manual |

**Checklist:**

| Item | Requirement | Verified |
|------|-------------|----------|
| Mounting rigid | No play or vibration | [ ] |
| Cable routing | No interference with motion | [ ] |
| Sensor orientation | Laser perpendicular to weld direction | [ ] |
| Look-ahead distance | 20-50mm ahead of TCP | [ ] |
| Working distance | Within sensor range | [ ] |

---

#### TEST-P4-H002: Sensor Accuracy Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H002 |
| **Component** | LaserProfiler |
| **Priority** | P0 |
| **Type** | Manual with Measurement |

**Equipment:**
- Calibrated step gauge

**Procedure:**

| Step | Nominal (mm) | Measured (mm) | Error (mm) | Pass |
|------|--------------|---------------|------------|------|
| 1 | 1.00 | | | [ ] < 0.05 |
| 2 | 2.00 | | | [ ] < 0.05 |
| 3 | 5.00 | | | [ ] < 0.05 |
| 4 | 10.00 | | | [ ] < 0.05 |

---

### 5.2 Seam Detection Hardware Tests

#### TEST-P4-H010: V-Groove Workpiece Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H010 |
| **Component** | Seam Detection |
| **Priority** | P0 |
| **Type** | Manual |

**Test Workpiece:**
- V-groove joint, 60°, 5mm depth

**Procedure:**

| Position | Nominal X (mm) | Detected X (mm) | Error (mm) | Pass |
|----------|----------------|-----------------|------------|------|
| 1 | 0.0 | | | [ ] < 0.5 |
| 2 | 0.0 | | | [ ] < 0.5 |
| 3 | 0.0 | | | [ ] < 0.5 |

**Repeatability:**
- [ ] 10 measurements, std dev < 0.2mm

---

#### TEST-P4-H011: Fillet Joint Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H011 |
| **Component** | Seam Detection |
| **Priority** | P0 |
| **Type** | Manual |

**Test Workpiece:**
- 90° fillet joint

**Procedure:**

| Test | Corner Position | Detected | Error | Pass |
|------|-----------------|----------|-------|------|
| X position | 0.0mm | ___mm | ___mm | [ ] |
| Z position | 0.0mm | ___mm | ___mm | [ ] |

---

### 5.3 Tracking Tests

#### TEST-P4-H020: Static Tracking Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H020 |
| **Component** | Seam Tracking |
| **Priority** | P0 |
| **Type** | Manual |

**Procedure:**
1. Position robot over known seam
2. Run tracking without motion
3. Verify stable detection

**Results:**

| Measurement | Value |
|-------------|-------|
| Detection rate | ___% (should be > 99%) |
| Mean position | X=___mm, Z=___mm |
| Std dev | X=___mm, Z=___mm |

**Pass Criteria:**
- [ ] Detection rate > 99%
- [ ] Position std dev < 0.2mm

---

#### TEST-P4-H021: Dynamic Tracking Test
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H021 |
| **Component** | Seam Tracking + Motion |
| **Priority** | P0 |
| **Type** | Manual |

**Procedure:**
1. Program straight line weld path
2. Offset workpiece 5mm from programmed path
3. Run weld with tracking enabled
4. Measure actual weld position

**Expected:**
- Weld should follow actual seam, not programmed path
- Tracking error < 1mm

**Results:**

| Position | Offset (mm) | Correction (mm) | Error (mm) |
|----------|-------------|-----------------|------------|
| Start | 5.0 | | |
| 25% | 5.0 | | |
| 50% | 5.0 | | |
| 75% | 5.0 | | |
| End | 5.0 | | |

---

#### TEST-P4-H022: Curved Seam Tracking
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-H022 |
| **Component** | Seam Tracking |
| **Priority** | P1 |
| **Type** | Manual |

**Test Workpiece:**
- Curved seam (radius 100mm)

**Procedure:**
1. Program straight line
2. Enable tracking
3. Execute and measure deviation

**Pass Criteria:**
- [ ] Robot follows curved seam
- [ ] Tracking error < 1mm throughout curve

---

## 6. System Tests

### 6.1 Milestone Verification

#### TEST-P4-S001: Milestone "Smart Weld"
| Field | Value |
|-------|-------|
| **ID** | TEST-P4-S001 |
| **Component** | All Phase 4 |
| **Priority** | P0 |
| **Type** | Manual System Test |

**Milestone Criteria Checklist:**

| # | Criterion | Test Reference | Result |
|---|-----------|----------------|--------|
| 1 | Laser profiler reads profile | TEST-P4-H001, H002 | [ ] |
| 2 | Seam detected automatically | TEST-P4-H010, H011 | [ ] |
| 3 | Seam tracking real-time works | TEST-P4-H020, H021 | [ ] |
| 4 | Position offset compensated | TEST-P4-H021 | [ ] |
| 5 | Hand-eye calibration < 1mm | TEST-P4-C001 | [ ] |

---

## 7. Performance Requirements

### 7.1 Tracking Performance

| Metric | Requirement | Test Method |
|--------|-------------|-------------|
| Update rate | > 100 Hz | TEST-P4-I010 |
| Latency | < 20ms | TEST-P4-I010 |
| Position accuracy | < 0.5mm | TEST-P4-H020 |
| Tracking error | < 1mm @ 10mm/s | TEST-P4-H021 |
| Detection rate | > 99% | TEST-P4-H020 |

---

## 8. Test Report Template

```
================================================================================
                         PHASE 4 TEST REPORT
================================================================================

Date: _______________
Tester: _______________
Build Version: _______________
Sensor Model: _______________

SUMMARY
-------
Unit Tests:         ___ / ___ passed
Integration Tests:  ___ / ___ passed
Calibration Tests:  ___ / ___ passed
Hardware Tests:     ___ / ___ passed

CALIBRATION RESULTS
-------------------
Hand-Eye Transform:
  X: ___mm  Y: ___mm  Z: ___mm
  Rx: ___°  Ry: ___°  Rz: ___°

Reprojection Error: ___mm

TRACKING PERFORMANCE
--------------------
Update Rate: ___Hz
Latency: ___ms
Position Accuracy: ___mm
Tracking Error: ___mm

MILESTONE STATUS
----------------
[ ] MILESTONE "Smart Weld" PASSED
[ ] MILESTONE "Smart Weld" FAILED

APPROVALS
---------
Tester: _______________
Date: _______________
================================================================================
```

---

## 9. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | System | Initial version |
