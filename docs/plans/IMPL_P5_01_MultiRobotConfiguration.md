# IMPL_P5_01: Multi-Robot Configuration System

## Overview

| Item | Value |
|------|-------|
| Plan ID | IMPL_P5_01 |
| Feature | Multi-Robot Configuration System |
| Priority | High |
| Estimated Effort | Large |
| Dependencies | P1_03 (ConfigLogging), P2_02 (Kinematics) |

## Objective

Implement a Robot Catalog system inspired by KUKA's configuration architecture, allowing the controller to support multiple robot types with different kinematic parameters (DH parameters, joint limits, etc.).

## Required Reading

Trước khi bắt đầu, ĐỌC các tài liệu sau:

1. `ressearch_doc_md/KUKA_Robot_Configuration_Architecture.md` - Kiến trúc cấu hình KUKA
2. `ressearch_doc_md/Robotics Library_ Robot Tùy Chỉnh & IK Giải Tích.md` - DH parameters và kinematics

## Current State Analysis

### Hiện tại đã có:
- `src/config/robot_config.yaml` - Single robot config (hardcoded cho 1 robot)
- `src/core/src/config/RobotConfig.hpp` - Struct cho robot config
- `src/core/src/config/ConfigManager.hpp` - Singleton config manager
- `src/core/src/kinematics/KinematicsService.hpp` - Đã có `setRobotConfig()`

### Cần thêm:
- Robot Catalog system (nhiều robot models)
- Robot Instance configuration (deployed robot với calibration)
- UI để chọn robot model
- Hot-reload kinematics khi đổi robot

---

## Architecture Design

### Proposed Directory Structure

```
src/config/
├── catalog/                         # Robot Model Catalog (read-only templates)
│   ├── catalog_index.yaml           # Index of all available robot models
│   │
│   ├── custom_6dof_1200/            # Custom robot model 1
│   │   ├── kinematic.yaml           # DH parameters
│   │   ├── limits.yaml              # Joint/velocity limits
│   │   └── info.yaml                # Metadata (name, manufacturer, reach, etc.)
│   │
│   ├── kuka_kr6_r900/               # KUKA-like robot model
│   │   ├── kinematic.yaml
│   │   ├── limits.yaml
│   │   └── info.yaml
│   │
│   └── ur5e/                        # UR5e-like robot model
│       ├── kinematic.yaml
│       ├── limits.yaml
│       └── info.yaml
│
├── instances/                       # Deployed Robot Instances
│   └── robot_r1/                    # Active robot instance
│       ├── instance.yaml            # Instance config (base_model + overrides)
│       └── calibration.yaml         # Calibration offsets
│
├── robot_config.yaml                # (Legacy - will migrate to catalog)
├── system_config.yaml               # System-level config
└── ui_config.json                   # UI config
```

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                          UI Layer                                │
│  ┌─────────────────┐     ┌─────────────────┐                    │
│  │ Robot Selector  │────►│ Configuration   │                    │
│  │ (Dropdown)      │     │ View            │                    │
│  └────────┬────────┘     └─────────────────┘                    │
│           │                                                      │
│           │ IPC: select_robot_model                             │
│           ▼                                                      │
├─────────────────────────────────────────────────────────────────┤
│                         Core Layer                               │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    ConfigManager                             ││
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     ││
│  │  │RobotCatalog │    │RobotInstance│    │SystemConfig │     ││
│  │  │  Manager    │───►│  Manager    │    │             │     ││
│  │  └──────┬──────┘    └──────┬──────┘    └─────────────┘     ││
│  │         │                   │                                ││
│  │         │ Load model        │ Create/update instance         ││
│  │         ▼                   ▼                                ││
│  │  ┌─────────────┐    ┌─────────────┐                         ││
│  │  │catalog/*.yaml│   │instances/*.yaml│                       ││
│  │  └─────────────┘    └─────────────┘                         ││
│  └─────────────────────────────────────────────────────────────┘│
│                              │                                   │
│                              │ setRobotConfig()                  │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                  KinematicsService                           ││
│  │  • ForwardKinematics                                         ││
│  │  • InverseKinematics                                         ││
│  │  • Updates DH parameters dynamically                         ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## Implementation Steps

### Phase 1: Core - Robot Catalog System

#### Step 1.1: Define Catalog File Schemas

**File:** `src/config/catalog/catalog_index.yaml`
```yaml
# Robot Catalog Index
version: "1.0"
models:
  - id: custom_6dof_1200
    name: "Custom 6DOF 1200mm"
    path: custom_6dof_1200/

  - id: kuka_kr6_r900
    name: "KUKA KR 6 R900"
    path: kuka_kr6_r900/

  - id: ur5e
    name: "Universal Robots UR5e"
    path: ur5e/
```

**File:** `src/config/catalog/custom_6dof_1200/info.yaml`
```yaml
id: custom_6dof_1200
name: "Custom 6DOF 1200mm"
manufacturer: "Custom"
model: "RC-6DOF-01"
type: "6-axis-articulated"
dof: 6
max_payload_kg: 6.0
reach_mm: 1200
mounting: ["floor", "ceiling", "wall"]
dh_convention: "modified"  # or "standard"
```

**File:** `src/config/catalog/custom_6dof_1200/kinematic.yaml`
```yaml
# DH Parameters (Modified DH Convention)
dh_parameters:
  - joint: 1
    a: 0
    alpha: 0
    d: 400
    theta_offset: 0

  - joint: 2
    a: 25
    alpha: -90
    d: 0
    theta_offset: -90

  # ... (6 joints total)

# Default TCP offset
default_tcp:
  x: 0
  y: 0
  z: 150
  rx: 0
  ry: 0
  rz: 0

# Home position
home_position: [0, -45, 90, 0, 45, 0]
```

**File:** `src/config/catalog/custom_6dof_1200/limits.yaml`
```yaml
joint_limits:
  - joint: 1
    min_deg: -170
    max_deg: 170
    max_velocity_deg_s: 120
    max_acceleration_deg_s2: 500

  - joint: 2
    min_deg: -120
    max_deg: 60
    max_velocity_deg_s: 120
    max_acceleration_deg_s2: 500

  # ... (6 joints total)
```

**Validation:**
- [ ] Tất cả catalog files tuân theo schema
- [ ] YAML syntax valid

---

#### Step 1.2: Create RobotCatalog C++ Classes

**File:** `src/core/src/config/RobotCatalog.hpp`

```cpp
#pragma once

#include "RobotConfig.hpp"
#include <map>
#include <optional>
#include <filesystem>

namespace robot_controller {
namespace config {

/**
 * Robot Model entry in catalog
 */
struct RobotModelInfo {
    std::string id;                   // e.g., "kuka_kr6_r900"
    std::string name;                 // e.g., "KUKA KR 6 R900"
    std::string manufacturer;
    std::string model;
    std::string type;
    int dof = 6;
    double maxPayloadKg = 0.0;
    double reachMm = 0.0;
    std::vector<std::string> mounting;
    std::string dhConvention = "modified";
    std::filesystem::path catalogPath;
};

/**
 * Full robot model (loaded from catalog)
 */
struct RobotModel {
    RobotModelInfo info;
    std::vector<DHParameter> dhParameters;
    std::vector<JointLimit> jointLimits;
    TcpOffset defaultTcp;
    std::array<double, 6> homePosition;
};

/**
 * Robot Catalog Manager
 * Loads and manages robot models from catalog directory
 */
class RobotCatalog {
public:
    /**
     * Load catalog from directory
     */
    bool loadCatalog(const std::filesystem::path& catalogDir);

    /**
     * Get list of available robot models
     */
    std::vector<RobotModelInfo> getAvailableModels() const;

    /**
     * Load full robot model by ID
     */
    std::optional<RobotModel> loadModel(const std::string& modelId) const;

    /**
     * Check if model exists
     */
    bool hasModel(const std::string& modelId) const;

    /**
     * Get model info by ID
     */
    std::optional<RobotModelInfo> getModelInfo(const std::string& modelId) const;

private:
    bool loadCatalogIndex(const std::filesystem::path& indexPath);
    std::optional<RobotModel> loadModelFromPath(const RobotModelInfo& info) const;

    std::filesystem::path m_catalogDir;
    std::map<std::string, RobotModelInfo> m_models;
};

} // namespace config
} // namespace robot_controller
```

**Validation:**
- [ ] Code compiles
- [ ] Unit test: Load catalog index
- [ ] Unit test: Load robot model by ID

---

#### Step 1.3: Create RobotInstance Manager

**File:** `src/config/instances/robot_r1/instance.yaml`
```yaml
# Robot Instance Configuration
instance_id: robot_r1
base_model: custom_6dof_1200
created: "2024-01-15"
last_modified: "2024-01-15"

# Overrides (optional - override values from base model)
overrides:
  tcp_offset:
    x: 0
    y: 0
    z: 200   # Different tool length
    rx: 0
    ry: 0
    rz: 0
```

**File:** `src/config/instances/robot_r1/calibration.yaml`
```yaml
# Calibration data for this specific robot instance
calibration_date: "2024-01-15"
calibrated_by: "Operator"

# Joint angle offsets (from factory calibration)
joint_offsets:
  - joint: 1
    offset_deg: 0.05
  - joint: 2
    offset_deg: -0.12
  - joint: 3
    offset_deg: 0.08
  - joint: 4
    offset_deg: 0.0
  - joint: 5
    offset_deg: -0.03
  - joint: 6
    offset_deg: 0.15

# DH corrections (optional - for precision applications)
dh_corrections: []
```

**File:** `src/core/src/config/RobotInstance.hpp`

```cpp
#pragma once

#include "RobotCatalog.hpp"
#include <filesystem>

namespace robot_controller {
namespace config {

/**
 * Calibration data for a robot instance
 */
struct CalibrationData {
    std::string calibrationDate;
    std::string calibratedBy;
    std::array<double, 6> jointOffsets = {0};
    // Future: DH corrections, coupling corrections
};

/**
 * Robot Instance - a deployed robot with specific calibration
 */
struct RobotInstance {
    std::string instanceId;
    std::string baseModelId;
    std::string created;
    std::string lastModified;

    // Merged config (base + overrides + calibration)
    RobotConfig config;
    CalibrationData calibration;
};

/**
 * Robot Instance Manager
 * Manages deployed robot instances
 */
class RobotInstanceManager {
public:
    explicit RobotInstanceManager(RobotCatalog& catalog);

    /**
     * Load instances from directory
     */
    bool loadInstances(const std::filesystem::path& instancesDir);

    /**
     * Get active robot instance
     */
    std::optional<RobotInstance> getActiveInstance() const;

    /**
     * Set active instance by ID
     */
    bool setActiveInstance(const std::string& instanceId);

    /**
     * Create new instance from model
     */
    bool createInstance(const std::string& instanceId,
                       const std::string& modelId);

    /**
     * Save instance configuration
     */
    bool saveInstance(const std::string& instanceId);

    /**
     * Get list of instances
     */
    std::vector<std::string> getInstanceIds() const;

private:
    RobotInstance mergeModelAndCalibration(
        const RobotModel& model,
        const CalibrationData& calibration,
        const TcpOffset& tcpOverride) const;

    RobotCatalog& m_catalog;
    std::filesystem::path m_instancesDir;
    std::map<std::string, RobotInstance> m_instances;
    std::string m_activeInstanceId;
};

} // namespace config
} // namespace robot_controller
```

**Validation:**
- [ ] Code compiles
- [ ] Unit test: Create instance from model
- [ ] Unit test: Apply calibration offsets

---

#### Step 1.4: Update ConfigManager

Modify existing `ConfigManager` to use new catalog system.

**File:** `src/core/src/config/ConfigManager.hpp` (Modified)

```cpp
// Add new members:
class ConfigManager {
public:
    // ... existing methods ...

    /**
     * Get robot catalog
     */
    RobotCatalog& robotCatalog() { return m_robotCatalog; }

    /**
     * Get instance manager
     */
    RobotInstanceManager& instanceManager() { return m_instanceManager; }

    /**
     * Select robot model (creates/updates active instance)
     */
    bool selectRobotModel(const std::string& modelId);

    /**
     * Get current active robot config (merged)
     */
    const RobotConfig& activeRobotConfig() const;

    /**
     * Event: Robot config changed
     */
    using ConfigChangedCallback = std::function<void(const RobotConfig&)>;
    void setConfigChangedCallback(ConfigChangedCallback callback);

private:
    RobotCatalog m_robotCatalog;
    RobotInstanceManager m_instanceManager{m_robotCatalog};
    ConfigChangedCallback m_configChangedCallback;
};
```

**Validation:**
- [ ] Backward compatible với existing code
- [ ] Unit test: selectRobotModel triggers callback

---

#### Step 1.5: Update KinematicsService Integration

Ensure kinematics updates when robot model changes.

**File:** `src/core/src/controller/RobotController.cpp` (Modified)

```cpp
// In initialize():
ConfigManager::instance().setConfigChangedCallback(
    [this](const RobotConfig& config) {
        // Convert RobotConfig to RobotKinematicConfig
        auto kinConfig = convertToKinematicConfig(config);
        m_kinematicsService->setRobotConfig(kinConfig);
        LOG_INFO("Robot kinematics updated for: {}", config.name);
    });
```

**Validation:**
- [ ] Kinematics updates when robot changes
- [ ] FK/IK correct after change

---

### Phase 2: IPC Layer - Robot Selection Commands

#### Step 2.1: Add IPC Message Types

**File:** `src/core/src/ipc/MessageTypes.hpp` (Add)

```cpp
// New message types
constexpr const char* MSG_GET_ROBOT_CATALOG = "get_robot_catalog";
constexpr const char* MSG_SELECT_ROBOT_MODEL = "select_robot_model";
constexpr const char* MSG_GET_ACTIVE_ROBOT = "get_active_robot";
constexpr const char* MSG_ROBOT_CONFIG_CHANGED = "robot_config_changed";
```

**File:** `src/core/src/ipc/CatalogPayloads.hpp` (New)

```cpp
#pragma once

#include <string>
#include <vector>

namespace robot_controller {
namespace ipc {

struct RobotModelSummary {
    std::string id;
    std::string name;
    std::string manufacturer;
    int dof;
    double maxPayloadKg;
    double reachMm;
};

struct GetRobotCatalogResponse {
    std::vector<RobotModelSummary> models;
    std::string activeModelId;
};

struct SelectRobotModelRequest {
    std::string modelId;
};

struct SelectRobotModelResponse {
    bool success;
    std::string error;
    RobotConfigData config;  // Full config after selection
};

// For pub/sub notification
struct RobotConfigChangedEvent {
    std::string modelId;
    std::string modelName;
    RobotConfigData config;
};

} // namespace ipc
} // namespace robot_controller
```

**Validation:**
- [ ] JSON serialization works
- [ ] Unit test: Serialize/deserialize payloads

---

#### Step 2.2: Register IPC Handlers

**File:** `src/core/src/controller/RobotController.cpp` (Modified)

```cpp
void RobotController::registerIpcHandlers() {
    // ... existing handlers ...

    // Get robot catalog
    m_ipcServer->registerHandler(MSG_GET_ROBOT_CATALOG,
        [this](const json& payload) -> json {
            return handleGetRobotCatalog(payload);
        });

    // Select robot model
    m_ipcServer->registerHandler(MSG_SELECT_ROBOT_MODEL,
        [this](const json& payload) -> json {
            return handleSelectRobotModel(payload);
        });

    // Get active robot
    m_ipcServer->registerHandler(MSG_GET_ACTIVE_ROBOT,
        [this](const json& payload) -> json {
            return handleGetActiveRobot(payload);
        });
}
```

**Validation:**
- [ ] IPC handlers work from UI
- [ ] Integration test: UI can get catalog list

---

### Phase 3: UI Layer - Robot Selection

#### Step 3.1: Add C# Service for Robot Catalog

**File:** `src/ui/RobotController.Common/Services/IRobotCatalogService.cs` (New)

```csharp
public interface IRobotCatalogService
{
    Task<List<RobotModelSummary>> GetAvailableModelsAsync();
    Task<RobotModelSummary?> GetActiveModelAsync();
    Task<bool> SelectModelAsync(string modelId);

    event EventHandler<RobotConfigChangedEventArgs>? ConfigChanged;
}

public class RobotModelSummary
{
    public string Id { get; set; } = "";
    public string Name { get; set; } = "";
    public string Manufacturer { get; set; } = "";
    public int Dof { get; set; }
    public double MaxPayloadKg { get; set; }
    public double ReachMm { get; set; }
}
```

**Validation:**
- [ ] Service connects to Core
- [ ] Can fetch catalog list

---

#### Step 3.2: Add Robot Selection ViewModel

**File:** `src/ui/RobotController.UI/ViewModels/RobotSelectionViewModel.cs` (New)

```csharp
public partial class RobotSelectionViewModel : ObservableObject
{
    private readonly IRobotCatalogService _catalogService;

    [ObservableProperty]
    private ObservableCollection<RobotModelSummary> _availableModels = new();

    [ObservableProperty]
    private RobotModelSummary? _selectedModel;

    [ObservableProperty]
    private bool _isLoading;

    [RelayCommand]
    private async Task LoadModelsAsync();

    [RelayCommand]
    private async Task SelectModelAsync();
}
```

**Validation:**
- [ ] ViewModel loads models
- [ ] Selection triggers Core update

---

#### Step 3.3: Add Robot Selection UI

**Option A: Add to Configuration Page**

Thêm section "Robot Model" vào ConfigurationView.xaml hiện có.

**Option B: New dedicated page**

Tạo RobotSetupView mới trong Views/Pages/.

**Recommended:** Option A (ít thay đổi hơn)

**File:** `src/ui/RobotController.UI/Views/Pages/ConfigurationView.xaml` (Modified)

```xml
<!-- Add Robot Model Section at top -->
<TextBlock Text="Robot Model" FontWeight="Bold" Margin="0,0,0,5"/>
<Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
    <StackPanel>
        <ComboBox ItemsSource="{Binding RobotModels}"
                  SelectedItem="{Binding SelectedRobotModel}"
                  DisplayMemberPath="Name"
                  Margin="0,0,0,10"/>

        <Grid>
            <Grid.ColumnDefinitions>
                <ColumnDefinition/>
                <ColumnDefinition/>
            </Grid.ColumnDefinitions>
            <StackPanel Grid.Column="0">
                <TextBlock Text="Manufacturer" Foreground="Gray" FontSize="10"/>
                <TextBlock Text="{Binding SelectedRobotModel.Manufacturer}"/>
            </StackPanel>
            <StackPanel Grid.Column="1">
                <TextBlock Text="Reach" Foreground="Gray" FontSize="10"/>
                <TextBlock Text="{Binding SelectedRobotModel.ReachMm, StringFormat={}{0} mm}"/>
            </StackPanel>
        </Grid>

        <Button Content="Apply" Command="{Binding ApplyRobotModelCommand}"
                Margin="0,10,0,0" HorizontalAlignment="Right"/>
    </StackPanel>
</Border>
```

**Validation:**
- [ ] ComboBox shows available models
- [ ] Apply button works
- [ ] 3D model updates when robot changes

---

#### Step 3.4: Update 3D Viewport

Khi robot model thay đổi, 3D visualization cần update.

**File:** `src/ui/RobotController.UI/Services/ViewportService.cs` (Modified)

```csharp
// Add method to reload robot model
public async Task ReloadRobotModelAsync(RobotConfigData config)
{
    // Clear existing model
    // Rebuild with new DH parameters
    await InitializeAsync(config);
    ModelUpdated?.Invoke(this, EventArgs.Empty);
}
```

**Validation:**
- [ ] 3D model reloads when robot changes
- [ ] Joint visualization correct

---

### Phase 4: Sample Robot Catalog

#### Step 4.1: Create Sample Robot Models

Create 3 sample robot models:

1. **custom_6dof_1200** - Current default robot (migrate existing config)
2. **kuka_kr6_r900** - KUKA KR 6 R900 style
3. **ur5e** - UR5e style (different kinematics)

**File locations:**
- `src/config/catalog/custom_6dof_1200/*.yaml`
- `src/config/catalog/kuka_kr6_r900/*.yaml`
- `src/config/catalog/ur5e/*.yaml`

**Validation:**
- [ ] All 3 models load correctly
- [ ] FK/IK correct for each model
- [ ] UI shows all models

---

## Testing Checklist

### Unit Tests
- [ ] `test_robot_catalog.cpp` - Catalog loading
- [ ] `test_robot_instance.cpp` - Instance management
- [ ] `test_config_migration.cpp` - Old config still works

### Integration Tests
- [ ] IPC: Get catalog list
- [ ] IPC: Select model
- [ ] Kinematics update after model change

### Manual Tests
- [ ] UI: Open Configuration page
- [ ] UI: See robot dropdown with 3 options
- [ ] UI: Select different robot
- [ ] UI: 3D model updates
- [ ] UI: Joint positions correct
- [ ] Save/restart: Selected robot persists

---

## Migration Plan

### Backward Compatibility

1. Keep `robot_config.yaml` as fallback
2. If no catalog found, use legacy config
3. Auto-migrate: Create instance from legacy config on first run

```cpp
bool ConfigManager::loadAll(const std::string& config_dir) {
    auto catalogPath = config_dir / "catalog";

    if (std::filesystem::exists(catalogPath)) {
        // New catalog system
        m_robotCatalog.loadCatalog(catalogPath);
        m_instanceManager.loadInstances(config_dir / "instances");
    } else {
        // Legacy fallback
        loadRobotConfig(config_dir / "robot_config.yaml");
    }
}
```

---

## Commit Milestones

| Milestone | Description | Files |
|-----------|-------------|-------|
| P5_01_M1 | Catalog file schemas + sample data | `src/config/catalog/**` |
| P5_01_M2 | RobotCatalog C++ classes | `src/core/src/config/RobotCatalog.*` |
| P5_01_M3 | RobotInstance manager | `src/core/src/config/RobotInstance.*` |
| P5_01_M4 | ConfigManager integration | `src/core/src/config/ConfigManager.*` |
| P5_01_M5 | IPC handlers | `src/core/src/ipc/*`, `src/core/src/controller/*` |
| P5_01_M6 | UI service + ViewModel | `src/ui/**` |
| P5_01_M7 | Configuration page UI | `Views/Pages/ConfigurationView.xaml` |
| P5_01_M8 | 3D viewport update | `Services/ViewportService.cs` |
| P5_01_M9 | Sample robot models | `src/config/catalog/**` |
| P5_01_M10 | Tests + documentation | `tests/*`, `docs/*` |

---

## Success Criteria

1. ✅ User có thể chọn robot từ dropdown trong UI
2. ✅ Core cập nhật kinematics khi đổi robot
3. ✅ 3D visualization hiển thị đúng robot mới
4. ✅ FK/IK correct cho tất cả robot models
5. ✅ Selection được lưu và persist qua restart
6. ✅ Backward compatible với config cũ

---

## Notes

- KUKA dùng binary format cho efficiency, ta dùng YAML cho readability
- Calibration là tính năng nâng cao, ban đầu có thể để trống
- Nên test với ít nhất 2 robot có kinematics khác nhau (e.g., KUKA vs UR)
