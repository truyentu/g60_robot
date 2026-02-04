# Phase 7: HMI Enhancement + Point Teaching - Design Document

| Metadata | Value |
|----------|-------|
| Document | Phase 7 Design Specification |
| Version | 1.0 |
| Created | 2026-02-04 |
| Status | Approved |

---

## 1. Overview

### 1.1 Scope

Phase 7 implements KUKA-inspired HMI enhancements and point teaching capabilities:

- Position Display với 3 coordinate modes (Joint/World/Base)
- Jog trong World/Base frame với Translation + Tool Axis Rotation
- Triple Override Control (Program/Jog/Manual)
- Point Teaching workflow
- Inline Forms cho 4 motion types (PTP/LIN/CIRC/SPLINE)
- Hierarchical Point Management (Folders → Programs → Points)

### 1.2 Goals

1. Operator có thể xem robot position trong multiple coordinate systems
2. Operator có thể jog robot trong World/Base frame
3. Operator có thể teach points trực tiếp từ current position
4. Operator có thể tạo motion programs với inline forms
5. Programs được tổ chức hierarchical (folders/programs/points)

---

## 2. Architecture

### 2.1 System Layers

```
┌─────────────────────────────────────────────────────┐
│                    UI Layer (WPF)                    │
│  ┌─────────────┐ ┌─────────────┐ ┌───────────────┐  │
│  │ Position    │ │ Jog Panel   │ │ Point Manager │  │
│  │ Display     │ │ (Enhanced)  │ │ (Tree View)   │  │
│  ├─────────────┤ ├─────────────┤ ├───────────────┤  │
│  │ Override    │ │ Inline Form │ │ Program       │  │
│  │ Panel       │ │ Editor      │ │ Editor        │  │
│  └─────────────┘ └─────────────┘ └───────────────┘  │
├─────────────────────────────────────────────────────┤
│                 ViewModel Layer                      │
│  PositionDisplayVM │ JogVM │ PointManagerVM │ etc.  │
├─────────────────────────────────────────────────────┤
│                 IPC Service Layer                    │
│  New messages: TEACH_POINT, GET_POINTS, JOG_FRAME   │
├─────────────────────────────────────────────────────┤
│                 C++ Core Layer                       │
│  ┌─────────────┐ ┌─────────────┐ ┌───────────────┐  │
│  │ JogService  │ │ PointStore  │ │ ProgramEngine │  │
│  │ (Enhanced)  │ │ (New)       │ │ (New)         │  │
│  └─────────────┘ └─────────────┘ └───────────────┘  │
└─────────────────────────────────────────────────────┘
```

---

## 3. Component Specifications

### 3.1 Position Display System

**Coordinate Modes:**

| Mode | Display | Source |
|------|---------|--------|
| Joint | J1-J6 (degrees) | Direct from encoders |
| World | X,Y,Z,Rx,Ry,Rz (mm/deg) | FK with Base = World |
| Base | X,Y,Z,Rx,Ry,Rz (mm/deg) | FK with Base = Active Base |

**StatusPayload Enhancement:**
```cpp
struct StatusPayload {
    std::vector<double> joints;      // J1-J6
    std::vector<double> tcpPosition; // World frame
    std::vector<double> tcpInBase;   // NEW: Position in active Base
    std::string activeBaseId;        // NEW
    std::string activeToolId;        // NEW
};
```

### 3.2 Enhanced Jog System

**Jog Modes:**

| Mode | Axes | Description |
|------|------|-------------|
| Joint | J1-J6 | Individual joint jog |
| World | X,Y,Z + Rz(tool) | TCP in World frame |
| Base | X,Y,Z + Rz(tool) | TCP in Active Base frame |

**New IPC Message:**
```cpp
struct JogCartesianRequest {
    std::string frame;      // "WORLD" | "BASE"
    double dx, dy, dz;      // Translation velocity (mm/s)
    double drz;             // Tool rotation velocity (deg/s)
};
```

**Algorithm:**
1. Get current TCP pose
2. Apply velocity in selected frame
3. Use Jacobian inverse for joint velocities
4. Send to motion controller

### 3.3 Triple Override Control

**Override Types:**

| Override | Range | Effect |
|----------|-------|--------|
| Program | 1-100% | Program execution speed |
| Jog | 1-100% | Manual jog speed |
| Manual | 1-100% | T1 mode cap (max 250mm/s) |

**Velocity Calculation:**
```
For T1 Mode:
  Effective = min(BaseVel × Override%, 250 × ManualOverride%)

For T2/AUTO Mode:
  Effective = BaseVel × Override%
```

### 3.4 Point Teaching System

**TaughtPoint Structure:**
```cpp
struct TaughtPoint {
    std::string id;           // "P001", "P002"
    std::string name;         // Friendly name
    std::array<double, 6> jointAngles;
    std::array<double, 6> cartesianWorld;
    std::array<double, 6> cartesianBase;
    std::string toolId;
    std::string baseId;
    MotionType defaultMotion; // PTP, LIN, CIRC, SPLINE
    std::string timestamp;
};
```

**Teaching Workflow:**
1. Jog robot to position
2. Press "Teach Point"
3. Enter name, select motion type
4. Point saved to current program

### 3.5 Inline Forms

**Motion Types:**

| Type | Parameters | Use Case |
|------|------------|----------|
| PTP | Target, Velocity% | Fast positioning |
| LIN | Target, Velocity(mm/s), Blend | Welding |
| CIRC | Via, Target, Velocity, Blend | Arc paths |
| SPLINE | Points[], Velocity | Smooth contours |

**MotionCommand Structure:**
```cpp
struct MotionCommand {
    MotionType type;
    std::string targetPointId;
    std::string viaPointId;      // For CIRC
    double velocity;
    double blendRadius;          // APO
    std::string toolId;
    std::string baseId;
};
```

### 3.6 Hierarchical Program Management

**Structure:**
```
📁 Projects/
├── 📁 Folder/
│   ├── 📄 Program.prog
│   │   ├── Points (P001, P002...)
│   │   └── Commands (motion sequence)
```

**File Format (.prog):**
```yaml
program:
  name: "MainProgram"
  created: "2026-02-04T10:30:00"

points:
  - id: "P001"
    name: "Home"
    joints: [0, -45, 90, 0, 45, 0]
    cartesian: [500, 0, 600, 180, 0, 0]

commands:
  - type: "PTP"
    target: "P001"
    velocity: 100
```

---

## 4. File Structure

### 4.1 C++ Core (New/Modified)

```
src/core/src/
├── jog/
│   └── JogService.hpp/cpp          # MODIFY: Add Cartesian jog
├── override/
│   └── OverrideManager.hpp/cpp     # NEW
├── program/
│   ├── TaughtPoint.hpp             # NEW
│   ├── MotionCommand.hpp           # NEW
│   ├── Program.hpp/cpp             # NEW
│   ├── ProgramStore.hpp/cpp        # NEW
│   └── ProgramEngine.hpp/cpp       # NEW (future execution)
└── ipc/
    ├── MessageTypes.hpp            # MODIFY: Add new types
    └── ProgramPayloads.hpp         # NEW
```

### 4.2 C# UI (New/Modified)

```
src/ui/RobotController.UI/
├── ViewModels/
│   ├── PositionDisplayViewModel.cs     # NEW
│   ├── JogViewModel.cs                 # MODIFY
│   ├── OverrideViewModel.cs            # NEW
│   ├── PointTeachingViewModel.cs       # NEW
│   ├── ProgramManagerViewModel.cs      # NEW
│   └── InlineFormViewModel.cs          # NEW
├── Views/Controls/
│   ├── PositionDisplay.xaml            # NEW
│   ├── JogPanel.xaml                   # MODIFY
│   ├── OverridePanel.xaml              # NEW
│   ├── ProgramTreeView.xaml            # NEW
│   └── InlineFormEditor.xaml           # NEW
└── Views/Dialogs/
    └── TeachPointDialog.xaml           # NEW
```

### 4.3 Messages (New)

```
C++ (MessageTypes.hpp):
- JOG_CARTESIAN
- SET_OVERRIDE / GET_OVERRIDE
- TEACH_POINT
- GET_PROGRAM_LIST / GET_PROGRAM
- CREATE_PROGRAM / UPDATE_PROGRAM / DELETE_PROGRAM
- CREATE_FOLDER / DELETE_FOLDER

C# (MessageTypes.cs):
- Same as above
```

---

## 5. Implementation Phases

### Phase 7.1: Position Display + Override
- Position display với 3 modes
- Override control panel
- StatusPayload enhancement

### Phase 7.2: Enhanced Jog
- Cartesian jog (World/Base frame)
- Tool rotation jog
- Jog mode selection UI

### Phase 7.3: Point Teaching
- TaughtPoint data structure
- Teach point dialog
- Point storage (in-memory first)

### Phase 7.4: Program Management
- Folder/Program hierarchy
- Program file format
- Tree view UI

### Phase 7.5: Inline Forms
- Motion command editor
- 4 motion types (PTP/LIN/CIRC/SPLINE)
- Command list in program

---

## 6. Success Criteria

1. ✓ Position displays correctly in Joint/World/Base modes
2. ✓ Jog works in World and Base coordinate frames
3. ✓ Override controls affect velocity correctly
4. ✓ Points can be taught and saved
5. ✓ Programs can be created/organized in folders
6. ✓ Motion commands can be added via inline forms
7. ✓ Programs saved/loaded from YAML files

---

## 7. Dependencies

- Phase 6 complete (Tool, Base, Modes)
- Kinematics service (FK/IK)
- Existing Jog infrastructure

---

## 8. Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Jacobian singularity during Cartesian jog | Detect singularity, switch to joint mode |
| Complex UI | Phased rollout, user testing |
| File corruption | Atomic writes, backup |

