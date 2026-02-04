# Implementation Plan: IMPL_P7_01 - Position Display & Override Control

## Overview

| Field | Value |
|-------|-------|
| Feature | Position Display + Override Control |
| Phase | 7.1 |
| Priority | P0 |
| Estimated Steps | 8 |

## Required Reading

- `docs/plans/2026-02-04-phase7-hmi-teaching-design.md` - Design document
- `src/core/src/kinematics/KinematicsService.hpp` - FK calculations
- `src/core/src/frame/BaseFrameManager.hpp` - Base frame transforms

## Dependencies

- Phase 6 complete (Base Frame, Operation Modes)
- Kinematics Service working

---

## Step 1: Enhance StatusPayload in C++

**Files:**
- `src/core/src/ipc/StatusPayloads.hpp` (modify)

**Changes:**
```cpp
// Add to StatusPayload
std::vector<double> tcpInBase;   // TCP in active base frame
std::string activeBaseId;
std::string activeToolId;
int programOverride;
int jogOverride;
int manualOverride;
```

**Validation:** File compiles

---

## Step 2: Create OverrideManager in C++

**Files:**
- `src/core/src/override/OverrideManager.hpp` (new)
- `src/core/src/override/OverrideManager.cpp` (new)

**Content:**
```cpp
class OverrideManager {
public:
    void setProgramOverride(int percent);
    void setJogOverride(int percent);
    void setManualOverride(int percent);

    int getProgramOverride() const;
    int getJogOverride() const;
    int getManualOverride() const;

    double applyOverride(double baseVelocity, OverrideContext ctx) const;

private:
    std::atomic<int> m_programOverride{100};
    std::atomic<int> m_jogOverride{100};
    std::atomic<int> m_manualOverride{100};
};
```

**Validation:** cmake --build build

---

## Step 3: Add Override IPC Handlers

**Files:**
- `src/core/src/ipc/MessageTypes.hpp` (modify)
- `src/core/src/ipc/OverridePayloads.hpp` (new)
- `src/core/src/controller/RobotController.cpp` (modify)

**New Messages:**
- `SET_OVERRIDE`
- `GET_OVERRIDE`

**Validation:** IPC messages work

---

## Step 4: Populate StatusPayload with new fields

**Files:**
- `src/core/src/controller/RobotController.cpp` (modify)

**Changes:**
- Calculate tcpInBase using BaseFrameManager
- Add activeBaseId, activeToolId
- Add override values

**Validation:** Status includes new fields

---

## Step 5: Add C# Override Types and Service

**Files:**
- `src/ui/RobotController.Common/Messages/OverridePayloads.cs` (new)
- `src/ui/RobotController.Common/Messages/StatusPayload.cs` (modify)
- `src/ui/RobotController.Common/Services/IIpcClientService.cs` (modify)
- `src/ui/RobotController.Common/Services/IpcClientService.cs` (modify)

**New Methods:**
- `SetOverrideAsync(int program, int jog, int manual)`
- `GetOverrideAsync()`

**Validation:** dotnet build

---

## Step 6: Create PositionDisplayViewModel

**Files:**
- `src/ui/RobotController.UI/ViewModels/PositionDisplayViewModel.cs` (new)

**Properties:**
- `DisplayMode` (Joint/World/Base)
- `Position1..6` (J1-J6 or X,Y,Z,Rx,Ry,Rz)
- `Labels1..6` (axis labels based on mode)

**Commands:**
- `SetDisplayModeCommand`

**Validation:** ViewModel created

---

## Step 7: Create PositionDisplay and OverridePanel Controls

**Files:**
- `src/ui/RobotController.UI/Views/Controls/PositionDisplay.xaml` (new)
- `src/ui/RobotController.UI/Views/Controls/PositionDisplay.xaml.cs` (new)
- `src/ui/RobotController.UI/Views/Controls/OverridePanel.xaml` (new)
- `src/ui/RobotController.UI/Views/Controls/OverridePanel.xaml.cs` (new)
- `src/ui/RobotController.UI/ViewModels/OverrideViewModel.cs` (new)

**Validation:** Controls render correctly

---

## Step 8: Integrate into MainWindow

**Files:**
- `src/ui/RobotController.UI/Views/MainWindow.xaml` (modify)
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs` (modify)

**Changes:**
- Add PositionDisplay control to Motion page
- Add OverridePanel to toolbar or status area
- Wire up ViewModels

**Validation:** Full integration test

---

## Files Summary

### New Files:
- `src/core/src/override/OverrideManager.hpp`
- `src/core/src/override/OverrideManager.cpp`
- `src/core/src/ipc/OverridePayloads.hpp`
- `src/ui/RobotController.Common/Messages/OverridePayloads.cs`
- `src/ui/RobotController.UI/ViewModels/PositionDisplayViewModel.cs`
- `src/ui/RobotController.UI/ViewModels/OverrideViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/PositionDisplay.xaml`
- `src/ui/RobotController.UI/Views/Controls/PositionDisplay.xaml.cs`
- `src/ui/RobotController.UI/Views/Controls/OverridePanel.xaml`
- `src/ui/RobotController.UI/Views/Controls/OverridePanel.xaml.cs`

### Modified Files:
- `src/core/src/ipc/StatusPayloads.hpp`
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/core/CMakeLists.txt`
- `src/ui/RobotController.Common/Messages/StatusPayload.cs`
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`
- `src/ui/RobotController.UI/Views/MainWindow.xaml`
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
