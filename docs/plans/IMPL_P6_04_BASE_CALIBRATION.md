# Implementation Plan: IMPL_P6_04 - Base/Workpiece Coordinate System

## Overview

| Field | Value |
|-------|-------|
| Feature | Base/Workpiece Coordinate Frame Management |
| Priority | P3 |
| Complexity | Medium |
| Estimated Steps | 12 |

## Required Reading

- `docs/features/KUKA_INSPIRED_FEATURES.md` - Feature specification
- `docs/01_ARCHITECTURE_OVERVIEW.md` - System architecture
- `src/core/src/kinematics/` - Kinematics module

## Dependencies

- Kinematics Service (existing)
- Tool Calibration (IMPL_P6_02) - Base frame depends on tool
- IPC Layer (existing)

---

## Phase 1: Core C++ - Base Frame Data Model

### Step 1.1: Define Base Frame Structures

**File**: `src/core/src/frame/FrameTypes.hpp`

```cpp
struct Frame {
    double x, y, z;       // mm - position
    double rx, ry, rz;    // degrees - orientation (Euler ZYX)

    Eigen::Matrix4d toMatrix() const;
    static Frame fromMatrix(const Eigen::Matrix4d& mat);
};

struct BaseFrame {
    std::string id;
    std::string name;
    std::string description;
    Frame frame;
    bool isActive;
};

enum class BaseCalibrationMethod {
    DIRECT_INPUT,
    THREE_POINT,      // Origin, +X, XY plane
    FOUR_POINT        // Origin, +X, +Y, +Z (overdetermined)
};

struct CalibrationPointData {
    int pointIndex;
    std::string pointName;  // "Origin", "X-Direction", "XY-Plane"
    std::array<double, 6> jointAngles;
    bool recorded;
};
```

**Validation**: File compiles

### Step 1.2: Create Base Frame Manager

**File**: `src/core/src/frame/BaseFrameManager.hpp`, `BaseFrameManager.cpp`

```cpp
class BaseFrameManager {
public:
    BaseFrameManager(KinematicsService& kinematics);

    // CRUD
    bool createBase(const BaseFrame& base);
    bool updateBase(const std::string& id, const BaseFrame& base);
    bool deleteBase(const std::string& id);
    std::optional<BaseFrame> getBase(const std::string& id) const;
    std::vector<BaseFrame> getAllBases() const;

    // Active base
    bool setActiveBase(const std::string& id);
    std::optional<BaseFrame> getActiveBase() const;
    std::string getActiveBaseId() const;

    // Coordinate transformation
    Frame transformToBase(const Frame& worldFrame) const;
    Frame transformToWorld(const Frame& baseFrame) const;

    // Calibration
    void startCalibration(BaseCalibrationMethod method);
    bool recordCalibrationPoint(int pointIndex);
    std::optional<Frame> calculateBaseFrame();
    void cancelCalibration();

    // Persistence
    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);

private:
    std::map<std::string, BaseFrame> m_bases;
    std::string m_activeBaseId;
    KinematicsService& m_kinematics;

    // Calibration state
    BaseCalibrationMethod m_calibMethod;
    std::vector<CalibrationPointData> m_calibPoints;
};
```

### Step 1.3: Implement 3-Point Base Calibration

**File**: `src/core/src/frame/BaseCalibration.hpp`, `BaseCalibration.cpp`

Algorithm:
1. Point 1: Origin của base frame
2. Point 2: Điểm trên trục +X
3. Point 3: Điểm trên mặt phẳng XY (phía +Y)

Tính toán:
```cpp
// X axis: từ P1 đến P2, normalize
Vector3d X = (P2 - P1).normalized();

// Temp Y: từ P1 đến P3
Vector3d tempY = (P3 - P1);

// Z axis: X cross tempY, normalize
Vector3d Z = X.cross(tempY).normalized();

// Y axis: Z cross X (đảm bảo orthogonal)
Vector3d Y = Z.cross(X);

// Build rotation matrix
Matrix3d R;
R.col(0) = X;
R.col(1) = Y;
R.col(2) = Z;

// Origin is P1
Vector3d origin = P1;
```

**Validation**: Unit test với known base frame

### Step 1.4: Integrate with Kinematics

**File**: `src/core/src/kinematics/KinematicsService.hpp/cpp` (modify)

- Add `setBaseFrame(const Frame& base)` method
- Modify FK to return position in base frame
- Modify IK to accept target in base frame

**Validation**: FK/IK correct in base frame

---

## Phase 2: Core C++ - Config Files

### Step 2.1: Create Base Frame Config Schema

**File**: `src/config/frames/world.yaml`

```yaml
base:
  id: "world"
  name: "World Frame"
  description: "Robot base coordinate system"
  frame:
    x: 0.0
    y: 0.0
    z: 0.0
    rx: 0.0
    ry: 0.0
    rz: 0.0
```

### Step 2.2: Create Sample Base Frames

**Files**:
- `src/config/frames/welding_table.yaml`
- `src/config/frames/conveyor.yaml`

**Validation**: YAML files parse correctly

---

## Phase 3: Core C++ - IPC Layer

### Step 3.1: Define IPC Message Types

**File**: `src/core/src/ipc/MessageTypes.hpp` (modify)

Add message types:
- `GET_BASE_LIST = 100`
- `GET_BASE = 101`
- `CREATE_BASE = 102`
- `UPDATE_BASE = 103`
- `DELETE_BASE = 104`
- `SELECT_BASE = 105`
- `GET_ACTIVE_BASE = 106`
- `START_BASE_CALIBRATION = 107`
- `RECORD_BASE_POINT = 108`
- `FINISH_BASE_CALIBRATION = 109`
- `BASE_CHANGED = 110` (event)

### Step 3.2: Create Base Payloads

**File**: `src/core/src/ipc/BasePayloads.hpp`

```cpp
struct GetBaseListResponse {
    std::vector<BaseFrame> bases;
    std::string activeBaseId;
};

struct StartBaseCalibrationRequest {
    std::string method;  // "3point", "direct"
};

struct RecordBasePointRequest {
    int pointIndex;  // 0=Origin, 1=X-dir, 2=XY-plane
};

struct RecordBasePointResponse {
    int pointIndex;
    std::string pointName;
    bool success;
    int totalPoints;
    int recordedPoints;
};

struct FinishBaseCalibrationResponse {
    bool success;
    Frame calculatedFrame;
    std::string error;
};
```

### Step 3.3: Register IPC Handlers

**File**: `src/core/src/controller/RobotController.cpp` (modify)

**Validation**: IPC test

---

## Phase 4: C# UI - Service Layer

### Step 4.1: Add C# Types and Payloads

**File**: `src/ui/RobotController.Common/Messages/MessageTypes.cs` (modify)
**File**: `src/ui/RobotController.Common/Messages/BasePayloads.cs` (new)

### Step 4.2: Add IPC Service Methods

**File**: `src/ui/RobotController.Common/Services/IIpcClientService.cs` (modify)
**File**: `src/ui/RobotController.Common/Services/IpcClientService.cs` (modify)

---

## Phase 5: C# UI - ViewModel and View

### Step 5.1: Create BaseFrameViewModel

**File**: `src/ui/RobotController.UI/ViewModels/BaseFrameViewModel.cs`

```csharp
public partial class BaseFrameViewModel : ObservableObject
{
    // Collections
    public ObservableCollection<BaseFrameItemViewModel> Bases { get; }

    // Selected/Active
    [ObservableProperty] BaseFrameItemViewModel? selectedBase;
    [ObservableProperty] BaseFrameItemViewModel? activeBase;

    // Editor
    [ObservableProperty] bool isEditing;
    [ObservableProperty] BaseFrame? editingBase;

    // Calibration state
    [ObservableProperty] bool isCalibrating;
    [ObservableProperty] string calibrationMethod;
    [ObservableProperty] int currentPointIndex;
    public ObservableCollection<CalibrationPointStatus> CalibrationPoints { get; }

    // Position display in active base frame
    [ObservableProperty] double posX, posY, posZ;
    [ObservableProperty] double rotRx, rotRy, rotRz;

    // Commands
    [RelayCommand] Task LoadBasesAsync();
    [RelayCommand] Task CreateBaseAsync();
    [RelayCommand] Task SaveBaseAsync();
    [RelayCommand] Task DeleteBaseAsync();
    [RelayCommand] Task SelectBaseAsync(BaseFrameItemViewModel base);
    [RelayCommand] Task StartCalibrationAsync(string method);
    [RelayCommand] Task RecordPointAsync(int pointIndex);
    [RelayCommand] Task FinishCalibrationAsync();
    [RelayCommand] void CancelCalibration();
}
```

### Step 5.2: Create BaseFramePanel View

**File**: `src/ui/RobotController.UI/Views/Controls/BaseFramePanel.xaml`

Layout:
```
+------------------------------------------+
| Base Frame Management                    |
+------------------------------------------+
| [Base List]         | [Base Details]     |
| - World*            | Name: _______      |
| - Welding Table     | Position:          |
| - Conveyor          |   X: ___ Y: ___    |
|                     |   Z: ___           |
| [+ New] [Delete]    | Orientation:       |
|                     |   RX: __ RY: __    |
|                     |   RZ: __           |
|                     | [Save] [Cancel]    |
+------------------------------------------+
| 3-Point Calibration                      |
| ● Origin (recorded at X:100 Y:50 Z:800) |
| ○ X-Direction (move to +X direction)    |
| ○ XY-Plane (move to +Y direction)       |
| [Record Current Position]                |
| [Calculate Base] [Cancel]                |
+------------------------------------------+
| Current Position in Active Base:         |
| X: 150.00  Y: 75.00  Z: 50.00           |
| RX: 0.00   RY: 180.00 RZ: 45.00         |
+------------------------------------------+
```

### Step 5.3: Create Base Calibration Wizard

**File**: `src/ui/RobotController.UI/Views/Dialogs/BaseCalibrationDialog.xaml`

Step-by-step wizard với hình ảnh minh họa:
1. Chọn method (3-point / direct input)
2. Step 1: Di chuyển TCP đến Origin, ghi điểm
3. Step 2: Di chuyển TCP theo hướng +X, ghi điểm
4. Step 3: Di chuyển TCP lên trên (hướng +Y từ origin), ghi điểm
5. Hiển thị kết quả và xác nhận

### Step 5.4: Integrate into Configuration View

**File**: `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml` (modify)

Add "Base Frames" tab

**Validation**: Full calibration workflow works

---

## Phase 6: Integration Testing

### Step 6.1: Base Frame CRUD Test

### Step 6.2: Calibration Test
1. Setup physical reference points
2. Run 3-point calibration
3. Verify calculated frame matches expected

### Step 6.3: Coordinate Transform Test
1. Set active base frame
2. Move robot
3. Verify position display in base coordinates is correct

---

## Files Summary

### New Files:
- `src/core/src/frame/FrameTypes.hpp`
- `src/core/src/frame/BaseFrameManager.hpp`
- `src/core/src/frame/BaseFrameManager.cpp`
- `src/core/src/frame/BaseCalibration.hpp`
- `src/core/src/frame/BaseCalibration.cpp`
- `src/core/src/ipc/BasePayloads.hpp`
- `src/config/frames/world.yaml`
- `src/config/frames/welding_table.yaml`
- `src/config/frames/conveyor.yaml`
- `src/ui/RobotController.Common/Messages/BasePayloads.cs`
- `src/ui/RobotController.UI/ViewModels/BaseFrameViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/BaseFramePanel.xaml`
- `src/ui/RobotController.UI/Views/Controls/BaseFramePanel.xaml.cs`
- `src/ui/RobotController.UI/Views/Dialogs/BaseCalibrationDialog.xaml`
- `src/ui/RobotController.UI/Views/Dialogs/BaseCalibrationDialog.xaml.cs`

### Modified Files:
- `src/core/CMakeLists.txt`
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/core/src/kinematics/KinematicsService.hpp/cpp`
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
- `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml`
