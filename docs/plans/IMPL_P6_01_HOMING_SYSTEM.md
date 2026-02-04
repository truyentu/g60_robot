# Implementation Plan: IMPL_P6_01 - Homing System

## Overview

| Field | Value |
|-------|-------|
| Feature | Robot Homing/Mastering System |
| Priority | P1 |
| Complexity | Medium |
| Estimated Steps | 12 |

## Required Reading

- `docs/features/KUKA_INSPIRED_FEATURES.md` - Feature specification
- `docs/01_ARCHITECTURE_OVERVIEW.md` - System architecture
- `src/core/src/motion/MotionController.hpp` - Motion controller interface

## Dependencies

- Motion Controller (existing)
- IPC Layer (existing)
- Safety Module (existing)

---

## Phase 1: Core C++ - Homing Service

### Step 1.1: Define Homing Types and States

**File**: `src/core/src/homing/HomingTypes.hpp`

```cpp
// Create enums for:
// - HomingMethod: LIMIT_SWITCH, INDEX_PULSE, MANUAL, ABSOLUTE_ENCODER
// - HomingState: NOT_HOMED, HOMING_IN_PROGRESS, HOMED, HOMING_ERROR
// - HomingDirection: POSITIVE, NEGATIVE
// Create HomingConfig struct with:
// - method, direction, velocity, acceleration
// - limit_switch_input, index_pulse_offset
// Create JointHomingStatus struct
```

**Validation**: File compiles without errors

### Step 1.2: Create Homing Service Class

**File**: `src/core/src/homing/HomingService.hpp`, `HomingService.cpp`

```cpp
class HomingService {
public:
    // Start homing for specific joint or all joints
    bool startHoming(int jointIndex = -1); // -1 = all joints
    bool stopHoming();

    // Query status
    HomingState getJointHomingState(int jointIndex) const;
    bool isAllJointsHomed() const;

    // Configuration
    void setHomingConfig(int jointIndex, const HomingConfig& config);
    HomingConfig getHomingConfig(int jointIndex) const;

    // Event callback
    using HomingCallback = std::function<void(int joint, HomingState state)>;
    void setHomingCallback(HomingCallback callback);

private:
    void homingLoop();
    void executeHomingSequence(int jointIndex);
};
```

**Validation**: Class compiles, unit test skeleton created

### Step 1.3: Implement Homing State Machine

**File**: `src/core/src/homing/HomingService.cpp`

Implement state machine:
1. IDLE → MOVING_TO_SWITCH (khi start)
2. MOVING_TO_SWITCH → BACKING_OFF (khi gặp switch)
3. BACKING_OFF → MOVING_SLOW (lùi ra khỏi switch)
4. MOVING_SLOW → HOMED (gặp switch lần 2 với tốc độ chậm)

**Validation**: Unit test cho state transitions

### Step 1.4: Integrate with Motion Controller

**File**: `src/core/src/motion/MotionController.hpp/cpp` (modify)

- Add method `moveToHomePosition(int joint, double velocity)`
- Add method `setJointZero(int joint)`
- Add callback for limit switch detection

**Validation**: Motion controller can execute homing moves

---

## Phase 2: Core C++ - IPC Layer

### Step 2.1: Define IPC Message Types

**File**: `src/core/src/ipc/MessageTypes.hpp` (modify)

Add message types:
- `START_HOMING = 60`
- `STOP_HOMING = 61`
- `GET_HOMING_STATE = 62`
- `HOMING_STATE_CHANGED = 63` (event)

### Step 2.2: Create Homing Payloads

**File**: `src/core/src/ipc/HomingPayloads.hpp`

```cpp
struct StartHomingRequest {
    int jointIndex;  // -1 = all
    std::string method;
};

struct HomingStateResponse {
    std::vector<JointHomingStatus> joints;
    bool allHomed;
};

struct HomingStateChangedEvent {
    int jointIndex;
    std::string state;
    std::string error;
};
```

### Step 2.3: Register IPC Handlers

**File**: `src/core/src/controller/RobotController.cpp` (modify)

Register handlers for:
- START_HOMING → call HomingService::startHoming()
- STOP_HOMING → call HomingService::stopHoming()
- GET_HOMING_STATE → return current homing state
- Publish HOMING_STATE_CHANGED events

**Validation**: Test IPC messages với mock client

---

## Phase 3: C# UI - Service Layer

### Step 3.1: Add C# Message Types

**File**: `src/ui/RobotController.Common/Messages/MessageTypes.cs` (modify)

Add enum values matching C++ side

### Step 3.2: Create C# Payloads

**File**: `src/ui/RobotController.Common/Messages/HomingPayloads.cs`

Create C# classes matching C++ structs

### Step 3.3: Add IPC Service Methods

**File**: `src/ui/RobotController.Common/Services/IIpcClientService.cs` (modify)
**File**: `src/ui/RobotController.Common/Services/IpcClientService.cs` (modify)

Add methods:
- `Task<bool> StartHomingAsync(int jointIndex = -1)`
- `Task<bool> StopHomingAsync()`
- `Task<HomingStateResponse> GetHomingStateAsync()`
- Event: `HomingStateChanged`

**Validation**: Service methods compile

---

## Phase 4: C# UI - ViewModel and View

### Step 4.1: Create HomingViewModel

**File**: `src/ui/RobotController.UI/ViewModels/HomingViewModel.cs`

```csharp
public partial class HomingViewModel : ObservableObject
{
    // Properties
    public ObservableCollection<JointHomingStatus> JointStates { get; }
    public bool IsHomingInProgress { get; }
    public bool IsAllHomed { get; }
    public string SelectedMethod { get; set; }

    // Commands
    [RelayCommand] Task StartHomingAllAsync();
    [RelayCommand] Task StartHomingSingleAsync(int joint);
    [RelayCommand] Task StopHomingAsync();
}
```

### Step 4.2: Create HomingPanel View

**File**: `src/ui/RobotController.UI/Views/Controls/HomingPanel.xaml`

UI Elements:
- Joint status list với indicators (Not Homed/Homing/Homed)
- Method selector dropdown
- "Home All" button
- Individual joint home buttons
- Stop button
- Progress indicators

### Step 4.3: Integrate into Main UI

**File**: `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml` (modify)

Add "Homing" tab with HomingPanel

**Validation**: UI displays and responds to mock data

---

## Phase 5: Integration Testing

### Step 5.1: End-to-End Test

1. Start UI
2. Connect to Core
3. Navigate to Configuration > Homing
4. Click "Home All"
5. Verify joints transition through states
6. Verify all joints show "Homed" when complete

### Step 5.2: Error Handling Test

1. Test homing với limit switch not triggered
2. Test stop during homing
3. Test homing when already homed

---

## Files Created/Modified Summary

### New Files:
- `src/core/src/homing/HomingTypes.hpp`
- `src/core/src/homing/HomingService.hpp`
- `src/core/src/homing/HomingService.cpp`
- `src/core/src/ipc/HomingPayloads.hpp`
- `src/ui/RobotController.Common/Messages/HomingPayloads.cs`
- `src/ui/RobotController.UI/ViewModels/HomingViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/HomingPanel.xaml`
- `src/ui/RobotController.UI/Views/Controls/HomingPanel.xaml.cs`

### Modified Files:
- `src/core/CMakeLists.txt`
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/core/src/motion/MotionController.hpp/cpp`
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
- `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml`
