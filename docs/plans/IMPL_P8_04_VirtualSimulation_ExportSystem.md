# IMPL_P8_04: Virtual Simulation - Export & Project System

| Metadata | Value |
|----------|-------|
| Phase | Phase 8: Virtual Simulation |
| Plan ID | IMPL_P8_04 |
| Title | Export & Project Management System |
| Priority | P2 |
| Status | PLANNED |
| Created | 2026-02-04 |
| Depends On | IMPL_P8_03 |

---

## 1. Overview

### 1.1 Mục tiêu

Xây dựng hệ thống export và project management cho:
- Save/Load programs (.rpl files)
- Export compiled programs (.json)
- Export point database (.points.yaml)
- Full project archive (.project.zip)

### 1.2 Use Cases

1. **Save Work** - Lưu chương trình đang viết để tiếp tục sau
2. **Share Programs** - Chia sẻ chương trình với đồng nghiệp
3. **Backup** - Backup toàn bộ project
4. **Transfer to Real Robot** - Export để deploy lên hardware thật

---

## 2. Implementation Steps

### Step 1: Define Export Formats

**Files to create:**
- `src/core/src/export/ExportFormats.hpp`

```cpp
#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace export_system {

// Program export (JSON)
struct ProgramExport {
    std::string name;
    std::string version;
    std::string created;
    std::string robot_model;
    std::string source_code;

    struct Instruction {
        int line;
        std::string type;
        std::string target;
        double velocity;
        std::string velocity_unit;
    };
    std::vector<Instruction> compiled_instructions;

    struct Metadata {
        double estimated_cycle_time_sec;
        double total_path_length_mm;
        int point_count;
    };
    Metadata metadata;

    nlohmann::json toJson() const;
    static ProgramExport fromJson(const nlohmann::json& j);
};

// Point export (YAML-compatible structure)
struct PointExport {
    std::string name;
    std::string type;  // "joint" or "cartesian"
    std::vector<double> values;
    std::string config;  // Robot configuration string
    std::string unit;

    nlohmann::json toJson() const;
};

struct PointsDatabase {
    std::string coordinate_system;
    std::vector<PointExport> points;

    std::string toYaml() const;
    static PointsDatabase fromYaml(const std::string& yaml);
};

// Full project
struct ProjectExport {
    std::string name;
    std::string robot_model;
    std::string created;

    std::string source_code;
    ProgramExport compiled;
    PointsDatabase points;

    // Robot config snapshot
    nlohmann::json robot_config;
    nlohmann::json tools;
    nlohmann::json bases;
};

} // namespace export_system
} // namespace robot_controller
```

---

### Step 2: Create ExportService

**Files to create:**
- `src/core/src/export/ExportService.hpp`
- `src/core/src/export/ExportService.cpp`

```cpp
#pragma once

#include "ExportFormats.hpp"
#include "../interpreter/AST.hpp"
#include <filesystem>

namespace robot_controller {
namespace export_system {

class ExportService {
public:
    // Export program source (.rpl)
    static bool exportSource(const std::string& source,
                             const std::filesystem::path& path);

    // Export compiled program (.json)
    static bool exportProgram(const interpreter::ProgramStmt& program,
                              const std::string& source,
                              const std::filesystem::path& path);

    // Export points (.points.yaml)
    static bool exportPoints(const PointsDatabase& points,
                             const std::filesystem::path& path);

    // Export full project (.project.zip)
    static bool exportProject(const ProjectExport& project,
                              const std::filesystem::path& path);

    // Import
    static std::string importSource(const std::filesystem::path& path);
    static std::optional<ProgramExport> importProgram(const std::filesystem::path& path);
    static std::optional<PointsDatabase> importPoints(const std::filesystem::path& path);
    static std::optional<ProjectExport> importProject(const std::filesystem::path& path);

private:
    static std::string getCurrentTimestamp();
};

} // namespace export_system
} // namespace robot_controller
```

**Implementation:**

```cpp
bool ExportService::exportProgram(const interpreter::ProgramStmt& program,
                                  const std::string& source,
                                  const std::filesystem::path& path)
{
    ProgramExport exp;
    exp.name = program.name;
    exp.version = "1.0";
    exp.created = getCurrentTimestamp();
    exp.source_code = source;

    // Compile to instructions
    for (const auto& stmt : program.body) {
        if (auto* motion = std::get_if<interpreter::MotionStmt>(&*stmt)) {
            ProgramExport::Instruction instr;
            instr.type = motion->type;
            // ... extract other fields
            exp.compiled_instructions.push_back(instr);
        }
    }

    // Calculate metadata
    // ... cycle time estimation, path length

    // Write JSON
    std::ofstream file(path);
    if (!file) return false;

    file << exp.toJson().dump(2);
    return true;
}

bool ExportService::exportProject(const ProjectExport& project,
                                  const std::filesystem::path& path)
{
    // Create temp directory
    auto tempDir = std::filesystem::temp_directory_path() / "robot_project";
    std::filesystem::create_directories(tempDir);

    // Write files
    std::ofstream(tempDir / "program.rpl") << project.source_code;
    std::ofstream(tempDir / "program.json") << project.compiled.toJson().dump(2);
    std::ofstream(tempDir / "points.yaml") << project.points.toYaml();
    std::ofstream(tempDir / "robot_config.json") << project.robot_config.dump(2);
    std::ofstream(tempDir / "tools.yaml") << project.tools.dump(2);
    std::ofstream(tempDir / "bases.yaml") << project.bases.dump(2);

    // Create metadata
    nlohmann::json metadata = {
        {"name", project.name},
        {"robot_model", project.robot_model},
        {"created", project.created},
        {"version", "1.0"}
    };
    std::ofstream(tempDir / "metadata.json") << metadata.dump(2);

    // Create ZIP (using miniz or similar)
    bool success = createZipFromDirectory(tempDir, path);

    // Cleanup
    std::filesystem::remove_all(tempDir);

    return success;
}
```

---

### Step 3: Add IPC Commands

**New message types:**

```cpp
// MessageTypes.hpp
EXPORT_PROGRAM,
EXPORT_POINTS,
EXPORT_PROJECT,
IMPORT_PROGRAM,
IMPORT_PROJECT
```

**Handlers:**

```cpp
m_ipcServer->registerHandler(MessageType::EXPORT_PROJECT,
    [this](const Message& request) -> nlohmann::json {
        std::string path = request.payload.value("path", "");

        ProjectExport project;
        project.name = m_currentProgram.name;
        project.robot_model = m_activeRobotPackage.name;
        project.source_code = m_programSource;
        project.compiled = createProgramExport(m_currentProgram, m_programSource);
        project.points = m_pointsDatabase;
        project.robot_config = robotToJson(m_activeRobotPackage);
        project.tools = toolsToJson();
        project.bases = basesToJson();

        bool success = ExportService::exportProject(project, path);

        return nlohmann::json{
            {"success", success},
            {"path", path}
        };
    });
```

---

### Step 4: Create Export UI

**Add to ProgramEditorViewModel:**

```csharp
[RelayCommand]
private async Task SaveProgramAsync()
{
    var dialog = new SaveFileDialog
    {
        Filter = "RPL Program|*.rpl|All Files|*.*",
        DefaultExt = ".rpl",
        FileName = ProgramName
    };

    if (dialog.ShowDialog() == true)
    {
        await _ipcClient.ExportProgramAsync(dialog.FileName, "source");
    }
}

[RelayCommand]
private async Task LoadProgramFromFileAsync()
{
    var dialog = new OpenFileDialog
    {
        Filter = "RPL Program|*.rpl|All Files|*.*"
    };

    if (dialog.ShowDialog() == true)
    {
        var source = await File.ReadAllTextAsync(dialog.FileName);
        ProgramSource = source;
        ProgramName = Path.GetFileNameWithoutExtension(dialog.FileName);
    }
}

[RelayCommand]
private async Task ExportProjectAsync()
{
    var dialog = new SaveFileDialog
    {
        Filter = "Robot Project|*.project.zip",
        DefaultExt = ".project.zip",
        FileName = $"{ProgramName}.project"
    };

    if (dialog.ShowDialog() == true)
    {
        await _ipcClient.ExportProjectAsync(dialog.FileName);
    }
}
```

**Add to toolbar:**

```xml
<Menu>
    <MenuItem Header="_File">
        <MenuItem Header="_New" Command="{Binding NewProgramCommand}"/>
        <MenuItem Header="_Open..." Command="{Binding LoadProgramFromFileCommand}"/>
        <MenuItem Header="_Save" Command="{Binding SaveProgramCommand}"/>
        <MenuItem Header="Save _As..." Command="{Binding SaveProgramAsCommand}"/>
        <Separator/>
        <MenuItem Header="_Export">
            <MenuItem Header="Export as JSON..." Command="{Binding ExportJsonCommand}"/>
            <MenuItem Header="Export Points..." Command="{Binding ExportPointsCommand}"/>
            <MenuItem Header="Export Project..." Command="{Binding ExportProjectCommand}"/>
        </MenuItem>
    </MenuItem>
</Menu>
```

---

### Step 5: Recent Files & Project History

**Add to app settings:**

```csharp
public class AppSettings
{
    public List<string> RecentFiles { get; set; } = new();
    public string LastProjectPath { get; set; } = "";
    public string LastRobotModel { get; set; } = "";
}
```

**Track recent files:**

```csharp
private void AddToRecentFiles(string path)
{
    _settings.RecentFiles.Remove(path);
    _settings.RecentFiles.Insert(0, path);
    if (_settings.RecentFiles.Count > 10)
        _settings.RecentFiles.RemoveAt(10);
    SaveSettings();
}
```

---

## 3. File Format Examples

### program.rpl (Source)

```
DEF WeldPath()
    ; Welding path for part A
    PTP HOME
    PTP P1 VEL=100%
    LIN P2 VEL=10mm/s
    LIN P3 VEL=10mm/s
    PTP HOME
END
```

### program.json (Compiled)

```json
{
  "program": {
    "name": "WeldPath",
    "version": "1.0",
    "created": "2026-02-04T10:30:00Z",
    "robot_model": "KUKA KR 6 R900"
  },
  "source_code": "DEF WeldPath()...",
  "compiled": {
    "instructions": [
      {"line": 3, "type": "PTP", "target": "HOME", "velocity": 100, "velocity_unit": "percent"},
      {"line": 4, "type": "PTP", "target": "P1", "velocity": 100, "velocity_unit": "percent"},
      {"line": 5, "type": "LIN", "target": "P2", "velocity": 10, "velocity_unit": "mm/s"}
    ]
  },
  "metadata": {
    "estimated_cycle_time_sec": 8.5,
    "total_path_length_mm": 650.3,
    "point_count": 4
  }
}
```

### points.yaml

```yaml
coordinate_system: world
points:
  HOME:
    type: joint
    values: [0, -90, 90, 0, 0, 0]
    unit: degrees

  P1:
    type: cartesian
    position: {x: 500, y: 0, z: 400}
    orientation: {rx: 0, ry: 90, rz: 0}
    config: "S1 T1 F1"
    unit: mm/degrees

  P2:
    type: cartesian
    position: {x: 500, y: 100, z: 400}
    orientation: {rx: 0, ry: 90, rz: 0}
    config: "S1 T1 F1"
```

---

## 4. Completion Checklist

- [ ] Step 1: Export format structures defined
- [ ] Step 2: ExportService implemented
- [ ] Step 3: IPC commands for export/import
- [ ] Step 4: Export UI (File menu, dialogs)
- [ ] Step 5: Recent files tracking
- [ ] Can save .rpl files
- [ ] Can export .json
- [ ] Can export .points.yaml
- [ ] Can export .project.zip
- [ ] Can import all formats

---

## 5. Summary

Phase 8 Virtual Simulation gồm 4 plans:

| Plan | Title | Purpose |
|------|-------|---------|
| IMPL_P8_01 | Robot Model Package | Import robot models với 3D meshes |
| IMPL_P8_02 | Program Interpreter | Parse và execute RPL programs |
| IMPL_P8_03 | Program Editor UI | Code editor với syntax highlighting |
| IMPL_P8_04 | Export System | Save/Load programs và projects |

Khi hoàn thành Phase 8, user có thể:
1. Import robot model với 3D mesh thực tế
2. Viết chương trình robot bằng RPL
3. Simulate chương trình trong 3D viewport
4. Teach points bằng jog + save
5. Export chương trình để sử dụng sau hoặc deploy lên robot thật
