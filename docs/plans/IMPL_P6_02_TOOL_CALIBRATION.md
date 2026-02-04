# Implementation Plan: IMPL_P6_02 - Tool Calibration System

## Overview

| Field | Value |
|-------|-------|
| Feature | Tool/TCP Management System |
| Priority | P2 |
| Complexity | Medium |
| Estimated Steps | 14 |

## Required Reading

- `docs/features/KUKA_INSPIRED_FEATURES.md` - Feature specification
- `docs/01_ARCHITECTURE_OVERVIEW.md` - System architecture
- `src/core/src/kinematics/` - Kinematics module

## Dependencies

- Kinematics Service (existing)
- IPC Layer (existing)
- Robot Catalog (IMPL_P5_01)

---

## Phase 1: Core C++ - Tool Data Model

### Step 1.1: Define Tool Data Structures

**File**: `src/core/src/tool/ToolTypes.hpp`

```cpp
struct ToolTCP {
    double x, y, z;       // mm - position offset
    double rx, ry, rz;    // degrees - orientation offset
};

struct ToolInertia {
    double mass;          // kg
    double cogX, cogY, cogZ;  // Center of Gravity (mm)
    double ixx, iyy, izz;     // Inertia tensor (kg*mm^2)
};

struct ToolData {
    std::string id;
    std::string name;
    std::string description;
    ToolTCP tcp;
    ToolInertia inertia;
    bool isActive;
};

enum class CalibrationMethod {
    DIRECT_INPUT,
    FOUR_POINT,
    SIX_POINT
};
```

**Validation**: File compiles

### Step 1.2: Create Tool Manager Class

**File**: `src/core/src/tool/ToolManager.hpp`, `ToolManager.cpp`

```cpp
class ToolManager {
public:
    // CRUD operations
    bool createTool(const ToolData& tool);
    bool updateTool(const std::string& id, const ToolData& tool);
    bool deleteTool(const std::string& id);
    std::optional<ToolData> getTool(const std::string& id) const;
    std::vector<ToolData> getAllTools() const;

    // Active tool
    bool setActiveTool(const std::string& id);
    std::optional<ToolData> getActiveTool() const;
    std::string getActiveToolId() const;

    // Calibration
    void startCalibration(CalibrationMethod method);
    bool recordCalibrationPoint(const std::array<double, 6>& jointAngles);
    std::optional<ToolTCP> calculateTCP();
    void cancelCalibration();

    // Persistence
    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);

private:
    std::map<std::string, ToolData> m_tools;
    std::string m_activeToolId;

    // Calibration state
    CalibrationMethod m_calibMethod;
    std::vector<std::array<double, 6>> m_calibPoints;
};
```

**Validation**: Unit test for CRUD operations

### Step 1.3: Implement 4-Point TCP Calibration

**File**: `src/core/src/tool/ToolCalibration.hpp`, `ToolCalibration.cpp`

Algorithm:
1. Thu thập 4 vị trí joint angles (TCP chạm cùng 1 điểm reference)
2. Tính Forward Kinematics cho mỗi vị trí → 4 flange positions
3. Giải hệ phương trình tìm TCP offset sao cho 4 TCP positions trùng nhau
4. Sử dụng Least Squares optimization

```cpp
class ToolCalibration {
public:
    static std::optional<ToolTCP> calculate4Point(
        const std::vector<Eigen::Matrix4d>& flangeFrames
    );

    static std::optional<ToolTCP> calculate6Point(
        const std::vector<Eigen::Matrix4d>& flangeFrames
    );
};
```

**Validation**: Unit test với known TCP offset

### Step 1.4: Integrate with Kinematics

**File**: `src/core/src/kinematics/KinematicsService.hpp/cpp` (modify)

- Add method `setToolOffset(const ToolTCP& tcp)`
- Modify Forward Kinematics để apply tool offset
- Modify Inverse Kinematics để account for tool

**Validation**: FK/IK results correct with tool offset

---

## Phase 2: Core C++ - Config Files

### Step 2.1: Create Tool Config Schema

**File**: `src/config/tools/tool_default.yaml`

```yaml
tool:
  id: "tool_default"
  name: "No Tool"
  description: "Default tool with no offset"
  tcp:
    x: 0.0
    y: 0.0
    z: 0.0
    rx: 0.0
    ry: 0.0
    rz: 0.0
  inertia:
    mass: 0.0
    cog: [0.0, 0.0, 0.0]
```

### Step 2.2: Create Sample Tool Files

**Files**:
- `src/config/tools/welding_torch.yaml`
- `src/config/tools/gripper_2finger.yaml`

**Validation**: YAML files parse correctly

---

## Phase 3: Core C++ - IPC Layer

### Step 3.1: Define IPC Message Types

**File**: `src/core/src/ipc/MessageTypes.hpp` (modify)

Add message types:
- `GET_TOOL_LIST = 70`
- `GET_TOOL = 71`
- `CREATE_TOOL = 72`
- `UPDATE_TOOL = 73`
- `DELETE_TOOL = 74`
- `SELECT_TOOL = 75`
- `GET_ACTIVE_TOOL = 76`
- `START_TCP_CALIBRATION = 77`
- `RECORD_CALIBRATION_POINT = 78`
- `FINISH_CALIBRATION = 79`
- `TOOL_CHANGED = 80` (event)

### Step 3.2: Create Tool Payloads

**File**: `src/core/src/ipc/ToolPayloads.hpp`

```cpp
struct GetToolListResponse {
    std::vector<ToolData> tools;
    std::string activeToolId;
};

struct SelectToolRequest {
    std::string toolId;
};

struct StartCalibrationRequest {
    std::string method;  // "4point", "6point", "direct"
};

struct RecordPointResponse {
    int pointIndex;
    int totalRequired;
    bool isComplete;
};

struct FinishCalibrationResponse {
    bool success;
    ToolTCP calculatedTcp;
    std::string error;
};
```

### Step 3.3: Register IPC Handlers

**File**: `src/core/src/controller/RobotController.cpp` (modify)

Register handlers for all tool-related messages

**Validation**: IPC round-trip test

---

## Phase 4: C# UI - Service Layer

### Step 4.1: Add C# Message Types and Payloads

**File**: `src/ui/RobotController.Common/Messages/MessageTypes.cs` (modify)
**File**: `src/ui/RobotController.Common/Messages/ToolPayloads.cs` (new)

### Step 4.2: Add IPC Service Methods

**File**: `src/ui/RobotController.Common/Services/IIpcClientService.cs` (modify)
**File**: `src/ui/RobotController.Common/Services/IpcClientService.cs` (modify)

Add methods:
- `Task<List<ToolData>> GetToolListAsync()`
- `Task<ToolData> GetToolAsync(string id)`
- `Task<bool> CreateToolAsync(ToolData tool)`
- `Task<bool> UpdateToolAsync(ToolData tool)`
- `Task<bool> DeleteToolAsync(string id)`
- `Task<bool> SelectToolAsync(string id)`
- `Task<ToolData> GetActiveToolAsync()`
- `Task StartCalibrationAsync(string method)`
- `Task<RecordPointResponse> RecordCalibrationPointAsync()`
- `Task<FinishCalibrationResponse> FinishCalibrationAsync()`
- Event: `ToolChanged`

---

## Phase 5: C# UI - ViewModel and View

### Step 5.1: Create ToolViewModel

**File**: `src/ui/RobotController.UI/ViewModels/ToolViewModel.cs`

```csharp
public partial class ToolViewModel : ObservableObject
{
    // Collections
    public ObservableCollection<ToolItemViewModel> Tools { get; }

    // Selected/Active
    [ObservableProperty] ToolItemViewModel? selectedTool;
    [ObservableProperty] ToolItemViewModel? activeTool;

    // Editor
    [ObservableProperty] bool isEditing;
    [ObservableProperty] ToolData? editingTool;

    // Calibration
    [ObservableProperty] bool isCalibrating;
    [ObservableProperty] int calibrationPointsRecorded;
    [ObservableProperty] int calibrationPointsRequired;
    [ObservableProperty] string calibrationMethod;

    // Commands
    [RelayCommand] Task LoadToolsAsync();
    [RelayCommand] Task CreateToolAsync();
    [RelayCommand] Task SaveToolAsync();
    [RelayCommand] Task DeleteToolAsync();
    [RelayCommand] Task SelectToolAsync(ToolItemViewModel tool);
    [RelayCommand] Task StartCalibrationAsync();
    [RelayCommand] Task RecordPointAsync();
    [RelayCommand] Task FinishCalibrationAsync();
    [RelayCommand] void CancelCalibration();
}
```

### Step 5.2: Create ToolPanel View

**File**: `src/ui/RobotController.UI/Views/Controls/ToolPanel.xaml`

Layout:
```
+------------------------------------------+
| Tool Management                          |
+------------------------------------------+
| [Tool List]        | [Tool Details]      |
| - Welding Torch*   | Name: _______       |
| - Gripper          | TCP X: ___ Y: ___   |
| - No Tool          |     Z: ___          |
|                    | Rot RX: __ RY: __   |
| [+ New] [Delete]   |     RZ: __          |
|                    | Mass: ___           |
|                    | [Save] [Cancel]     |
+------------------------------------------+
| TCP Calibration                          |
| Method: [4-Point ▼]                      |
| Points: ●●○○ (2/4 recorded)             |
| [Record Point] [Calculate] [Cancel]      |
+------------------------------------------+
```

### Step 5.3: Create Calibration Wizard Dialog

**File**: `src/ui/RobotController.UI/Views/Dialogs/TcpCalibrationDialog.xaml`

Step-by-step wizard:
1. Select calibration method
2. Instructions for each point
3. Record point button
4. Progress indicator
5. Result display
6. Accept/Reject

### Step 5.4: Integrate into Main UI

**File**: `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml` (modify)

Add "Tools" tab with ToolPanel

**Validation**: UI displays tools and calibration workflow works

---

## Phase 6: Integration Testing

### Step 6.1: Tool CRUD Test
1. Create new tool with manual TCP input
2. Select tool as active
3. Verify kinematics uses new TCP
4. Edit tool
5. Delete tool

### Step 6.2: Calibration Test
1. Start 4-point calibration
2. Move robot to 4 positions (same TCP point)
3. Record each point
4. Calculate TCP
5. Verify result is reasonable

---

## Files Created/Modified Summary

### New Files:
- `src/core/src/tool/ToolTypes.hpp`
- `src/core/src/tool/ToolManager.hpp`
- `src/core/src/tool/ToolManager.cpp`
- `src/core/src/tool/ToolCalibration.hpp`
- `src/core/src/tool/ToolCalibration.cpp`
- `src/core/src/ipc/ToolPayloads.hpp`
- `src/config/tools/tool_default.yaml`
- `src/config/tools/welding_torch.yaml`
- `src/config/tools/gripper_2finger.yaml`
- `src/ui/RobotController.Common/Messages/ToolPayloads.cs`
- `src/ui/RobotController.UI/ViewModels/ToolViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/ToolPanel.xaml`
- `src/ui/RobotController.UI/Views/Controls/ToolPanel.xaml.cs`
- `src/ui/RobotController.UI/Views/Dialogs/TcpCalibrationDialog.xaml`
- `src/ui/RobotController.UI/Views/Dialogs/TcpCalibrationDialog.xaml.cs`

### Modified Files:
- `src/core/CMakeLists.txt`
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/core/src/kinematics/KinematicsService.hpp`
- `src/core/src/kinematics/KinematicsService.cpp`
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
- `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml`
