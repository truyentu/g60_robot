# Test Architecture for Robot Controller

## Overview

Test suite được thiết kế để verify từng feature trong FEATURE_LIST.md hoạt động đúng.

## Test Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Integration Tests                         │
│              (Full system E2E verification)                  │
├─────────────────────────────────────────────────────────────┤
│                    Component Tests                           │
│         (IPC, Cross-module communication)                    │
├──────────────────────────┬──────────────────────────────────┤
│     C++ Unit Tests       │       C# Unit Tests              │
│     (Google Test)        │       (xUnit)                    │
│  - Core Platform         │    - ViewModels                  │
│  - State Machine         │    - Services                    │
│  - Kinematics            │    - IPC Client                  │
│  - Trajectory            │    - 3D Viewport                 │
│  - Motion Controller     │                                  │
│  - Welding               │                                  │
│  - Vision                │                                  │
│  - Safety                │                                  │
└──────────────────────────┴──────────────────────────────────┘
```

## Test Categories

| Category | Framework | Location | Features Tested |
|----------|-----------|----------|-----------------|
| Core Platform | GTest | `src/core/tests/test_core_platform.cpp` | CP001-CP010 |
| State Machine | GTest | `src/core/tests/test_state_machine.cpp` | SM001-SM008 |
| Safety | GTest | `src/core/tests/test_safety.cpp` | SF001-SF007 |
| Kinematics | GTest | `src/core/tests/test_kinematics.cpp` | KN001-KN006 |
| Trajectory | GTest | `src/core/tests/test_trajectory.cpp` | TJ001-TJ008 |
| Motion Controller | GTest | `src/core/tests/test_motion_controller.cpp` | MC001-MC005 |
| Firmware | GTest | `src/core/tests/test_firmware.cpp` | FW001-FW006 |
| Welding | GTest | `src/core/tests/test_welding.cpp` | WD001-WD011 |
| Weaving | GTest | `src/core/tests/test_weaving.cpp` | WV001-WV008 |
| Vision | GTest | `src/core/tests/test_vision.cpp` | VS001-VS012 |
| HMI ViewModels | xUnit | `src/ui/RobotController.Tests/` | HM001-HVS007 |

## Feature ID Mapping

Mỗi test case PHẢI có comment ghi Feature ID để trace được:

```cpp
// Feature: CP001 - ConfigManager
TEST_F(CorePlatformTest, CP001_ConfigManager_LoadsYaml) { ... }

// Feature: SF001 - SafetyMonitor
TEST_F(SafetyTest, SF001_SafetyMonitor_DetectsDualChannelMismatch) { ... }
```

## Test Naming Convention

```
{FeatureID}_{ComponentName}_{TestScenario}
```

Examples:
- `CP001_ConfigManager_LoadsRobotConfig`
- `SM003_StateMachine_TransitionsToEStop`
- `SF002_DualChannel_Detects50msTimeout`
- `KN002_InverseKinematics_Returns8Solutions`

## Running Tests

```bash
# Run all C++ tests
cd src/core/build
ctest --output-on-failure

# Run specific test file
./robot_controller_tests --gtest_filter="SafetyTest.*"

# Run specific feature tests
./robot_controller_tests --gtest_filter="*SF001*"

# Run C# tests
cd src/ui
dotnet test --filter "Category=FeatureTest"

# Generate coverage report
ctest -T Coverage
```

## Test Report Integration

Tests output được ghi vào:
- `src/core/build/test_results.xml` (JUnit format)
- `docs/TEST_RESULTS.md` (Human readable)

Script tự động update FEATURE_VERIFICATION_REPORT.md dựa trên test results.
