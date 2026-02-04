# IMPL_P8_01: Virtual Simulation - Robot Model Package System

| Metadata | Value |
|----------|-------|
| Phase | Phase 8: Virtual Simulation |
| Plan ID | IMPL_P8_01 |
| Title | Robot Model Package System |
| Priority | P0 (Foundation for simulation) |
| Status | PLANNED |
| Created | 2026-02-04 |

---

## 1. Overview

### 1.1 Mục tiêu

Xây dựng hệ thống để import và quản lý robot models với 3D mesh thực tế, cho phép simulation mà không cần hardware.

### 1.2 Scope

- Robot Model Package format (.robotpkg)
- Import robot từ package hoặc built-in library
- Load 3D STL meshes cho visualization
- Validate DH parameters và joint limits

### 1.3 Out of Scope

- Program interpreter (IMPL_P8_02)
- Program editor UI (IMPL_P8_03)
- Export system (IMPL_P8_04)

---

## 2. Prerequisites

- [ ] Phase 1-7 hoàn thành
- [ ] Helix3D đã hoạt động với simplified geometry
- [ ] Robot Catalog system (IMPL_P5_01) đã có

---

## 3. Required Reading

- `docs/KUKA_INSPIRED_FEATURES.md` - Section 3.6 (HMI Features)
- `ressearch_doc_md/KUKA_Reference.md` - Robot configuration

---

## 4. Implementation Steps

### Step 1: Define Robot Package Schema

**Files to create:**
- `src/core/src/config/RobotPackageSchema.hpp`

**Content:**

```cpp
#pragma once

#include <string>
#include <vector>
#include <array>
#include <optional>

namespace robot_controller {
namespace config {

struct JointMeshInfo {
    std::string visual_mesh;      // Path to visual STL
    std::string collision_mesh;   // Path to collision STL (optional)
    std::array<double, 3> origin; // Mesh origin offset
};

struct JointDefinition {
    std::string name;             // A1, A2, etc.
    std::string type;             // "revolute" or "prismatic"

    // DH Parameters
    double dh_a;                  // Link length (mm)
    double dh_alpha;              // Link twist (degrees)
    double dh_d;                  // Link offset (mm)
    double dh_theta_offset;       // Joint angle offset (degrees)

    // Limits
    double limit_min;             // degrees or mm
    double limit_max;
    double velocity_max;          // deg/s or mm/s
    double acceleration_max;

    // Mesh
    JointMeshInfo mesh;
};

struct RobotPackage {
    // Metadata
    std::string name;
    std::string manufacturer;
    std::string model_type;       // "6-axis-industrial", etc.
    double payload_kg;
    double reach_mm;

    // Kinematics
    std::string dh_convention;    // "modified_dh" or "standard_dh"
    std::vector<JointDefinition> joints;

    // Base
    std::string base_mesh;
    std::array<double, 6> base_origin;  // x, y, z, rx, ry, rz

    // Flange
    std::array<double, 3> flange_offset;  // x, y, z from last joint

    // Default positions
    std::vector<double> home_position;

    // Package path (for loading meshes)
    std::string package_path;
};

} // namespace config
} // namespace robot_controller
```

**Validation:**
- Header compiles without errors

---

### Step 2: Create Robot Package Loader

**Files to create:**
- `src/core/src/config/RobotPackageLoader.hpp`
- `src/core/src/config/RobotPackageLoader.cpp`

**Header:**

```cpp
#pragma once

#include "RobotPackageSchema.hpp"
#include <optional>
#include <filesystem>

namespace robot_controller {
namespace config {

class RobotPackageLoader {
public:
    /**
     * Load robot package from directory
     * @param packagePath Path to robot package directory
     * @return RobotPackage if successful
     */
    static std::optional<RobotPackage> loadFromDirectory(
        const std::filesystem::path& packagePath);

    /**
     * Load robot package from ZIP file
     * @param zipPath Path to .robotpkg.zip file
     * @return RobotPackage if successful
     */
    static std::optional<RobotPackage> loadFromZip(
        const std::filesystem::path& zipPath);

    /**
     * Validate robot package
     * @param package Package to validate
     * @return Error message if invalid, empty if valid
     */
    static std::string validate(const RobotPackage& package);

    /**
     * Get list of built-in robot packages
     * @return Vector of package names
     */
    static std::vector<std::string> getBuiltInPackages();

    /**
     * Load built-in robot package by name
     * @param name Package name (e.g., "kuka_kr6_r900")
     * @return RobotPackage if found
     */
    static std::optional<RobotPackage> loadBuiltIn(const std::string& name);

private:
    static bool parseRobotYaml(const std::filesystem::path& yamlPath,
                               RobotPackage& package);
};

} // namespace config
} // namespace robot_controller
```

**Implementation key parts:**

```cpp
std::optional<RobotPackage> RobotPackageLoader::loadFromDirectory(
    const std::filesystem::path& packagePath)
{
    // Check robot.yaml exists
    auto yamlPath = packagePath / "robot.yaml";
    if (!std::filesystem::exists(yamlPath)) {
        LOG_ERROR("robot.yaml not found in {}", packagePath.string());
        return std::nullopt;
    }

    RobotPackage package;
    package.package_path = packagePath.string();

    if (!parseRobotYaml(yamlPath, package)) {
        return std::nullopt;
    }

    // Validate
    auto error = validate(package);
    if (!error.empty()) {
        LOG_ERROR("Package validation failed: {}", error);
        return std::nullopt;
    }

    return package;
}
```

**Validation:**
```bash
# Build Core
cd src/core/build && cmake --build . --config Debug
```

---

### Step 3: Create Built-in Robot Library

**Files to create:**
- `src/config/robots/kuka_kr6_r900/robot.yaml`
- `src/config/robots/kuka_kr6_r900/meshes/visual/*.stl` (placeholder)

**robot.yaml example:**

```yaml
name: "KUKA KR 6 R900"
manufacturer: "KUKA"
type: "6-axis-industrial"
payload_kg: 6
reach_mm: 901

kinematics:
  convention: "modified_dh"
  joints:
    - name: "A1"
      type: "revolute"
      dh: { a: 25, alpha: -90, d: 400, theta_offset: 0 }
      limits: { min: -170, max: 170, vel_max: 360, accel_max: 720 }
      mesh: { visual: "meshes/visual/link1.stl" }

    - name: "A2"
      type: "revolute"
      dh: { a: 455, alpha: 0, d: 0, theta_offset: -90 }
      limits: { min: -190, max: 45, vel_max: 300, accel_max: 600 }
      mesh: { visual: "meshes/visual/link2.stl" }

    # ... (remaining joints)

home_position: [0, -90, 90, 0, 0, 0]

base:
  mesh: "meshes/visual/base.stl"
  origin: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }

flange:
  offset: { x: 0, y: 0, z: 80 }
```

**Validation:**
- YAML parses correctly
- Package loader can load built-in

---

### Step 4: Update ViewportService for STL Loading

**Files to modify:**
- `src/ui/RobotController.UI/Services/ViewportService.cs`

**Add STL loading capability:**

```csharp
using HelixToolkit.Wpf;

public async Task<bool> LoadRobotPackageAsync(RobotPackage package)
{
    _robotPackage = package;
    _linkModels.Clear();

    // Load base mesh
    if (!string.IsNullOrEmpty(package.BaseMesh))
    {
        var basePath = Path.Combine(package.PackagePath, package.BaseMesh);
        var baseMesh = await LoadStlMeshAsync(basePath);
        if (baseMesh != null)
        {
            _baseModel = baseMesh;
        }
    }

    // Load link meshes
    foreach (var joint in package.Joints)
    {
        if (!string.IsNullOrEmpty(joint.Mesh?.Visual))
        {
            var meshPath = Path.Combine(package.PackagePath, joint.Mesh.Visual);
            var linkMesh = await LoadStlMeshAsync(meshPath);
            _linkModels.Add(joint.Name, linkMesh);
        }
    }

    // Update DH parameters in kinematics
    UpdateKinematicsFromPackage(package);

    return true;
}

private async Task<Model3D?> LoadStlMeshAsync(string path)
{
    return await Task.Run(() =>
    {
        if (!File.Exists(path))
        {
            Log.Warning("Mesh file not found: {Path}", path);
            return null;
        }

        var reader = new StLReader();
        var model = reader.Read(path);
        return model;
    });
}
```

**Validation:**
- STL files load without crash
- Robot displays with real meshes

---

### Step 5: Add IPC Commands for Package Management

**Files to modify:**
- `src/core/src/ipc/MessageTypes.hpp`
- `src/core/src/controller/RobotController.cpp`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`

**New message types:**

```cpp
// MessageTypes.hpp
LOAD_ROBOT_PACKAGE,
GET_ROBOT_PACKAGE_INFO,
GET_AVAILABLE_PACKAGES
```

**Handlers:**

```cpp
// GET_AVAILABLE_PACKAGES
m_ipcServer->registerHandler(MessageType::GET_AVAILABLE_PACKAGES,
    [](const Message& request) -> nlohmann::json {
        auto packages = RobotPackageLoader::getBuiltInPackages();
        return nlohmann::json{
            {"success", true},
            {"packages", packages}
        };
    });

// LOAD_ROBOT_PACKAGE
m_ipcServer->registerHandler(MessageType::LOAD_ROBOT_PACKAGE,
    [this](const Message& request) -> nlohmann::json {
        std::string packageName = request.payload.value("package_name", "");

        auto package = RobotPackageLoader::loadBuiltIn(packageName);
        if (!package) {
            return nlohmann::json{{"success", false}, {"error", "Package not found"}};
        }

        // Update active robot
        m_activeRobotPackage = *package;

        return nlohmann::json{
            {"success", true},
            {"package", packageToJson(*package)}
        };
    });
```

**Validation:**
- IPC commands work from UI
- Package info returned correctly

---

### Step 6: Create Robot Package Browser UI

**Files to create:**
- `src/ui/RobotController.UI/ViewModels/RobotPackageBrowserViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/RobotPackageBrowser.xaml`

**ViewModel:**

```csharp
public partial class RobotPackageBrowserViewModel : ObservableObject
{
    [ObservableProperty]
    private ObservableCollection<RobotPackageInfo> _availablePackages = new();

    [ObservableProperty]
    private RobotPackageInfo? _selectedPackage;

    [RelayCommand]
    private async Task LoadPackagesAsync()
    {
        var packages = await _ipcClient.GetAvailablePackagesAsync();
        AvailablePackages.Clear();
        foreach (var pkg in packages)
        {
            AvailablePackages.Add(pkg);
        }
    }

    [RelayCommand]
    private async Task SelectPackageAsync()
    {
        if (SelectedPackage == null) return;

        var result = await _ipcClient.LoadRobotPackageAsync(SelectedPackage.Name);
        if (result.Success)
        {
            await _viewportService.LoadRobotPackageAsync(result.Package);
        }
    }
}
```

**XAML:**

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.RobotPackageBrowser">
    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <TextBlock Text="ROBOT LIBRARY" FontWeight="Bold" Margin="0,0,0,10"/>

            <ListBox ItemsSource="{Binding AvailablePackages}"
                     SelectedItem="{Binding SelectedPackage}"
                     Height="200">
                <ListBox.ItemTemplate>
                    <DataTemplate>
                        <StackPanel Orientation="Horizontal">
                            <Image Source="{Binding Thumbnail}" Width="48" Height="48"/>
                            <StackPanel Margin="10,0">
                                <TextBlock Text="{Binding Name}" FontWeight="Bold"/>
                                <TextBlock Text="{Binding Manufacturer}" Foreground="Gray"/>
                            </StackPanel>
                        </StackPanel>
                    </DataTemplate>
                </ListBox.ItemTemplate>
            </ListBox>

            <Button Content="Load Robot"
                    Command="{Binding SelectPackageCommand}"
                    Margin="0,10,0,0"/>
        </StackPanel>
    </Border>
</UserControl>
```

**Validation:**
- UI shows available packages
- Selecting package loads robot in viewport

---

## 5. Completion Checklist

- [ ] Step 1: RobotPackageSchema.hpp created
- [ ] Step 2: RobotPackageLoader implemented
- [ ] Step 3: Built-in robot library created (at least 1 robot)
- [ ] Step 4: ViewportService loads STL meshes
- [ ] Step 5: IPC commands for package management
- [ ] Step 6: Robot Package Browser UI
- [ ] All builds pass (Core + UI)
- [ ] Can load robot package and see realistic 3D model

---

## 6. Test Cases

| Test | Expected Result |
|------|-----------------|
| Load built-in package | Robot appears with meshes |
| Load invalid package | Error message shown |
| Switch between packages | Viewport updates correctly |
| Package without meshes | Fallback to simplified geometry |

---

## 7. Next Steps

After completing this plan:
- IMPL_P8_02: Program Interpreter (Lexer/Parser/Executor)
- IMPL_P8_03: Program Editor UI
- IMPL_P8_04: Export System
