# Implementation Plan: IMPL_P6_03 - Operation Mode Management

## Overview

| Field | Value |
|-------|-------|
| Feature | Operation Mode Management (Manual/Test/Auto/Remote) |
| Priority | P5 |
| Complexity | Low-Medium |
| Estimated Steps | 10 |

## Required Reading

- `docs/features/KUKA_INSPIRED_FEATURES.md` - Feature specification
- `docs/01_ARCHITECTURE_OVERVIEW.md` - System architecture
- `src/core/src/safety/` - Safety module

## Dependencies

- Safety Module (existing)
- State Machine (existing)
- IPC Layer (existing)

---

## Phase 1: Core C++ - Operation Mode System

### Step 1.1: Define Operation Modes

**File**: `src/core/src/mode/OperationMode.hpp`

```cpp
enum class OperationMode {
    MANUAL,     // T1 - Teaching mode, max 250mm/s, deadman required
    TEST,       // T2 - Test mode, full speed, operator supervision
    AUTO,       // Automatic mode, full speed, safety interlocks
    REMOTE      // External control (PLC), full safety system
};

struct OperationModeConfig {
    double maxLinearVelocity;      // mm/s
    double maxJointVelocity;       // deg/s
    bool requireDeadman;           // Deadman switch required
    bool requireSafetyFence;       // Safety fence must be closed
    bool allowExternalControl;     // PLC can control robot
    bool reducedWorkspace;         // Use reduced workspace limits
};

struct ModeTransitionRequirement {
    bool robotStopped;
    bool safetyFenceClosed;
    bool deadmanReleased;
    bool noActiveAlarms;
    bool homingComplete;
};

// Mode configuration constants
const std::map<OperationMode, OperationModeConfig> MODE_CONFIGS = {
    {OperationMode::MANUAL, {250.0, 30.0, true, false, false, false}},
    {OperationMode::TEST, {2000.0, 180.0, false, false, false, true}},
    {OperationMode::AUTO, {2000.0, 180.0, false, true, false, false}},
    {OperationMode::REMOTE, {2000.0, 180.0, false, true, true, false}}
};
```

**Validation**: File compiles

### Step 1.2: Create Mode Manager Class

**File**: `src/core/src/mode/ModeManager.hpp`, `ModeManager.cpp`

```cpp
class ModeManager {
public:
    ModeManager(SafetyController& safety, MotionController& motion);

    // Mode control
    OperationMode getCurrentMode() const;
    bool requestModeChange(OperationMode newMode);
    bool canTransitionTo(OperationMode newMode) const;

    // Get transition requirements
    ModeTransitionRequirement getTransitionRequirements(OperationMode target) const;
    std::vector<std::string> getMissingRequirements(OperationMode target) const;

    // Get mode config
    OperationModeConfig getCurrentModeConfig() const;
    double getMaxVelocity() const;

    // Callback
    using ModeChangeCallback = std::function<void(OperationMode oldMode, OperationMode newMode)>;
    void setModeChangeCallback(ModeChangeCallback callback);

private:
    OperationMode m_currentMode = OperationMode::MANUAL;
    SafetyController& m_safety;
    MotionController& m_motion;
    ModeChangeCallback m_callback;

    bool checkTransitionRequirements(OperationMode target) const;
    void applyModeConfig(OperationMode mode);
};
```

### Step 1.3: Implement Mode Transitions

**File**: `src/core/src/mode/ModeManager.cpp`

Implementation:
1. Validate current state (robot stopped, no errors)
2. Check mode-specific requirements
3. Apply velocity limits to motion controller
4. Update safety controller settings
5. Fire mode change event

**Validation**: Unit test for mode transitions

### Step 1.4: Integrate with Safety and Motion

**File**: `src/core/src/safety/SafetyController.hpp/cpp` (modify)

- Add `setOperationMode(OperationMode mode)` method
- Adjust safety checks based on mode

**File**: `src/core/src/motion/MotionController.hpp/cpp` (modify)

- Add velocity limiting based on mode
- Check deadman switch in MANUAL mode

**Validation**: Velocity limits applied correctly per mode

---

## Phase 2: Core C++ - IPC Layer

### Step 2.1: Define IPC Message Types

**File**: `src/core/src/ipc/MessageTypes.hpp` (modify)

Add message types:
- `GET_OPERATION_MODE = 90`
- `SET_OPERATION_MODE = 91`
- `GET_MODE_REQUIREMENTS = 92`
- `OPERATION_MODE_CHANGED = 93` (event)

### Step 2.2: Create Mode Payloads

**File**: `src/core/src/ipc/ModePayloads.hpp`

```cpp
struct GetOperationModeResponse {
    std::string mode;           // "MANUAL", "TEST", "AUTO", "REMOTE"
    double maxLinearVelocity;
    double maxJointVelocity;
    bool deadmanRequired;
};

struct SetOperationModeRequest {
    std::string mode;
};

struct SetOperationModeResponse {
    bool success;
    std::string newMode;
    std::string error;
    std::vector<std::string> missingRequirements;
};

struct GetModeRequirementsResponse {
    std::string targetMode;
    bool robotStopped;
    bool safetyFenceClosed;
    bool deadmanReleased;
    bool noActiveAlarms;
    bool homingComplete;
    std::vector<std::string> missingItems;
};

struct OperationModeChangedEvent {
    std::string previousMode;
    std::string newMode;
};
```

### Step 2.3: Register IPC Handlers

**File**: `src/core/src/controller/RobotController.cpp` (modify)

**Validation**: IPC test

---

## Phase 3: C# UI - Service Layer

### Step 3.1: Add C# Types and Payloads

**File**: `src/ui/RobotController.Common/Messages/MessageTypes.cs` (modify)
**File**: `src/ui/RobotController.Common/Messages/ModePayloads.cs` (new)

### Step 3.2: Add IPC Service Methods

**File**: `src/ui/RobotController.Common/Services/IIpcClientService.cs` (modify)
**File**: `src/ui/RobotController.Common/Services/IpcClientService.cs` (modify)

Add:
- `Task<GetOperationModeResponse> GetOperationModeAsync()`
- `Task<SetOperationModeResponse> SetOperationModeAsync(string mode)`
- `Task<GetModeRequirementsResponse> GetModeRequirementsAsync(string mode)`
- Event: `OperationModeChanged`

---

## Phase 4: C# UI - ViewModel and View

### Step 4.1: Create ModeViewModel

**File**: `src/ui/RobotController.UI/ViewModels/ModeViewModel.cs`

```csharp
public partial class ModeViewModel : ObservableObject
{
    // Current mode
    [ObservableProperty] string currentMode = "MANUAL";
    [ObservableProperty] Brush modeColor = Brushes.Yellow;
    [ObservableProperty] double maxVelocity;
    [ObservableProperty] bool deadmanRequired;

    // Mode selection
    public ObservableCollection<ModeOption> AvailableModes { get; }
    [ObservableProperty] ModeOption? selectedMode;

    // Requirements
    [ObservableProperty] bool canSwitchMode;
    public ObservableCollection<string> MissingRequirements { get; }

    // Commands
    [RelayCommand] Task RefreshModeAsync();
    [RelayCommand] Task SwitchModeAsync(string mode);
    [RelayCommand] Task CheckRequirementsAsync(string mode);
}

public class ModeOption
{
    public string Name { get; set; }
    public string Description { get; set; }
    public Brush Color { get; set; }
    public string Icon { get; set; }
}
```

### Step 4.2: Create ModeSelector Control

**File**: `src/ui/RobotController.UI/Views/Controls/ModeSelector.xaml`

Layout - Radio button style với màu sắc:
```
+----------------------------------------+
| Operation Mode                          |
+----------------------------------------+
| ○ MANUAL    [Yellow]   Teaching mode   |
|   Max: 250mm/s, Deadman required       |
+----------------------------------------+
| ○ TEST      [Orange]   Test mode       |
|   Full speed, Operator supervision     |
+----------------------------------------+
| ○ AUTO      [Green]    Automatic       |
|   Full speed, Safety fence required    |
+----------------------------------------+
| ○ REMOTE    [Blue]     External ctrl   |
|   PLC control, Full safety system      |
+----------------------------------------+
| Requirements for selected mode:        |
| ✓ Robot stopped                        |
| ✗ Safety fence closed                  |
| [Switch Mode]                          |
+----------------------------------------+
```

### Step 4.3: Add Mode Indicator to Status Bar

**File**: `src/ui/RobotController.UI/Views/MainWindow.xaml` (modify)

Add mode indicator in status bar:
- Current mode name
- Color-coded background
- Click to open mode selector

### Step 4.4: Integrate ModeViewModel

**File**: `src/ui/RobotController.UI/ViewModels/MainViewModel.cs` (modify)

Add `ModeViewModel` property and initialize it

**Validation**: Mode switching works through UI

---

## Phase 5: Integration Testing

### Step 5.1: Mode Switch Test
1. Start in MANUAL mode
2. Stop robot, clear alarms
3. Switch to TEST mode
4. Verify velocity limits increased
5. Switch back to MANUAL

### Step 5.2: Requirement Validation Test
1. Try to switch to AUTO while fence open
2. Verify error message and missing requirements shown
3. Close fence (simulated)
4. Switch succeeds

---

## Files Created/Modified Summary

### New Files:
- `src/core/src/mode/OperationMode.hpp`
- `src/core/src/mode/ModeManager.hpp`
- `src/core/src/mode/ModeManager.cpp`
- `src/core/src/ipc/ModePayloads.hpp`
- `src/ui/RobotController.Common/Messages/ModePayloads.cs`
- `src/ui/RobotController.UI/ViewModels/ModeViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/ModeSelector.xaml`
- `src/ui/RobotController.UI/Views/Controls/ModeSelector.xaml.cs`

### Modified Files:
- `src/core/CMakeLists.txt`
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/core/src/safety/SafetyController.hpp/cpp`
- `src/core/src/motion/MotionController.hpp/cpp`
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
- `src/ui/RobotController.UI/Views/MainWindow.xaml`
