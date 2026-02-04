# KUKA-Inspired Features Documentation

| Metadata | Value |
|----------|-------|
| Document | KUKA-Inspired Features Tracking |
| Version | 1.0 |
| Created | 2026-02-04 |
| Status | Active |

---

## 1. Overview

Document này track các features đã được inspired từ KUKA robot controller và implement vào Robot Controller project. Mục đích là để:
1. Ghi nhận các features đã implement
2. So sánh với KUKA original
3. Xác định các features có thể thêm trong tương lai

---

## 2. Features Đã Implement

### 2.1 Homing/Mastering System

| Aspect | KUKA Original | Our Implementation | Status |
|--------|---------------|-------------------|--------|
| Per-joint mastering | BCO (Block Coincidence) mastering | Per-joint homing với multiple methods | ✓ Done |
| All-axis mastering | "Master all axes" function | All-joint homing command | ✓ Done |
| Mastering methods | EMD, dial gauge, reference point | LIMIT_SWITCH, INDEX_PULSE, ABSOLUTE_ENCODER | ✓ Done |
| Visual feedback | SmartPAD display | HomingPanel với status per joint | ✓ Done |
| Homing state persistence | $MAMES (mastered flags) | Config persistence | ✓ Done |

**UI Location:** Configuration → Homing tab

**Files:**
- `src/core/src/homing/HomingService.hpp/cpp`
- `src/ui/RobotController.UI/ViewModels/HomingViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/HomingPanel.xaml`

---

### 2.2 Tool (TCP) Management

| Aspect | KUKA Original | Our Implementation | Status |
|--------|---------------|-------------------|--------|
| Tool definition | TOOL_DATA[1..16] | ToolData với unlimited tools | ✓ Done |
| TCP parameters | X, Y, Z, A, B, C | X, Y, Z, Rx, Ry, Rz (mm/degrees) | ✓ Done |
| Tool mass/inertia | LOAD_DATA | ToolInertia (mass) | ✓ Done |
| Active tool selection | $TOOL = TOOL_DATA[n] | SelectToolAsync() | ✓ Done |
| 4-point calibration | XYZ 4-Point method | 4-point TCP calibration | ✓ Done |
| 6-point calibration | XYZ 6-Point (orientation) | 6-point calibration | ✓ Done |
| Tool persistence | $config.dat | YAML config files | ✓ Done |

**UI Location:** Configuration → Tools tab

**Files:**
- `src/core/src/tool/ToolManager.hpp/cpp`
- `src/core/src/tool/ToolCalibration.hpp/cpp`
- `src/ui/RobotController.UI/ViewModels/ToolViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/ToolPanel.xaml`

---

### 2.3 Base Frame (Workpiece Coordinate System)

| Aspect | KUKA Original | Our Implementation | Status |
|--------|---------------|-------------------|--------|
| Base definition | BASE_DATA[1..32] | BaseFrameData với unlimited bases | ✓ Done |
| Frame parameters | X, Y, Z, A, B, C | X, Y, Z, Rx, Ry, Rz (mm/degrees) | ✓ Done |
| Active base selection | $BASE = BASE_DATA[n] | SelectBaseAsync() | ✓ Done |
| 3-point calibration | 3-Point method | Origin, X-dir, XY-plane | ✓ Done |
| 4-point calibration | 4-Point (indirect) | Overdetermined calibration | ✓ Done |
| World frame | BASE_DATA[0] = World | "world" as default base | ✓ Done |
| Base persistence | $config.dat | YAML config files | ✓ Done |

**UI Location:** Configuration → Base Frames tab

**Files:**
- `src/core/src/frame/BaseFrameManager.hpp/cpp`
- `src/core/src/frame/BaseCalibration.hpp/cpp`
- `src/ui/RobotController.UI/ViewModels/BaseFrameViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/BaseFramePanel.xaml`

---

### 2.4 Operation Modes

| Aspect | KUKA Original | Our Implementation | Status |
|--------|---------------|-------------------|--------|
| T1 Mode | Manual reduced velocity (250 mm/s) | MANUAL mode, 250 mm/s limit | ✓ Done |
| T2 Mode | Test mode, full velocity | TEST mode, 1000 mm/s limit | ✓ Done |
| AUT Mode | Automatic operation | AUTO mode, 2000 mm/s limit | ✓ Done |
| AUT EXT Mode | External/Remote control | REMOTE mode, 2000 mm/s limit | ✓ Done |
| Mode switching | Key switch + software | Software-based với requirements | ✓ Done |
| Velocity enforcement | Per-mode limits | Velocity limit per mode | ✓ Done |
| Mode requirements | Safety conditions | Transition requirements check | ✓ Done |

**UI Location:** Main toolbar (ModeSelector)

**Files:**
- `src/core/src/mode/ModeManager.hpp/cpp`
- `src/core/src/mode/OperationMode.hpp`
- `src/ui/RobotController.UI/ViewModels/ModeViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/ModeSelector.xaml`

---

### 2.5 Robot Configuration (Multi-Robot)

| Aspect | KUKA Original | Our Implementation | Status |
|--------|---------------|-------------------|--------|
| Robot catalog | WorkVisual project | Robot Catalog (YAML) | ✓ Done |
| DH parameters | $machine.dat | DHParameters in config | ✓ Done |
| Joint limits | $machine.dat | JointLimits per model | ✓ Done |
| Model selection | Project deployment | SelectRobotModelAsync() | ✓ Done |
| Instance management | Robot instances | RobotInstance với calibration | ✓ Done |

**UI Location:** Configuration → Robot Catalog tab

**Files:**
- `src/core/src/config/RobotCatalog.hpp/cpp`
- `src/core/src/config/RobotInstance.hpp/cpp`
- `src/ui/RobotController.UI/ViewModels/RobotCatalogViewModel.cs`
- `src/ui/RobotController.UI/Views/Controls/RobotCatalogPanel.xaml`

---

## 3. Features Chưa Implement (Candidates for Future)

### 3.1 Motion Features

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Spline motion | SPLINE/SLIN/SCIRC | Smooth path với continuous velocity, C2 continuity | High | High |
| Constant path velocity | $VEL.CP | Velocity control along Cartesian path | High | Medium |
| Approximate positioning | $APO | Corner rounding/blending để tránh dừng tại waypoints | High | Medium |
| Geometric operator | : operator | Offset frames relative to base/tool | Medium | Low |
| Advance run | $ADVANCE | Look-ahead motion planning | Medium | High |
| External axes | E1-E6 | Control thêm 6 external axes (positioner, track) | Medium | High |
| Conveyor tracking | ConveyorTech | Synchronization với moving conveyor | Low | High |

### 3.2 Safety Features

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Cartesian space monitoring | SafeRobot | Define safe/forbidden zones trong Cartesian space | High | Medium |
| Safe velocity monitoring | Safe velocity | Real-time TCP velocity check against limits | High | Low |
| Axis monitoring | Axis range monitoring | Monitor joint positions against soft limits | High | Low |
| Safe tool monitoring | Safe tool | Tool orientation limits (để tránh va chạm) | Medium | Medium |
| Reduced workspace | Reduced mode | Limited motion area khi cửa mở | Medium | Medium |
| Collision detection | Collision detection | Force/torque monitoring phát hiện va chạm | Low | High |
| Safe standstill | SLS/SS1/SS2 | Safety-rated stopping functions | Low | High |

### 3.3 Programming Features

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| **Inline forms** | Inline forms | GUI templates cho motion commands (PTP, LIN, CIRC) | **Critical** | Medium |
| **Point teaching** | Touch-up | Teach points trực tiếp bằng jog + save | **Critical** | Low |
| Point list management | Point management | CRUD operations cho taught points | High | Low |
| Program structure | FOLD/ENDFOLD | Collapsible code blocks | High | Low |
| Program editor | KRL Editor | Text-based program editing với syntax highlight | Medium | Medium |
| Submit interpreter | Submit | Background task execution (parallel processing) | Medium | High |
| Interrupt handling | INTERRUPT DECL | Event-driven programming với priority | Medium | High |
| Trigger functions | TRIGGER | Distance/time-based I/O switching | Medium | Medium |
| Wait functions | WAIT FOR | Conditional waiting | Low | Low |

### 3.4 I/O & Communication

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Digital I/O | $IN[1..4096]/$OUT[1..4096] | Boolean I/O signals | High | Low |
| Analog I/O | $ANIN[1..32]/$ANOUT[1..32] | Analog signals (0-10V, 4-20mA) | Medium | Low |
| Signal aliasing | SIGNAL | Tên gợi nhớ cho I/O (SIGNAL gripper $OUT[5]) | Medium | Low |
| EtherNet/IP | EtherNet/IP | Industrial Ethernet protocol | Medium | High |
| PROFINET | PROFINET | Siemens industrial protocol | Medium | High |
| Modbus TCP | Modbus | Simple industrial protocol | Medium | Medium |
| OPC-UA | mxAutomation | Industry 4.0 interface | Low | High |

### 3.5 Welding Features (ArcTech)

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Arc sensing | ArcSense | Through-arc seam tracking (TAST) | High | High |
| Touch sensing | TouchSense | Wire touch detection để tìm workpiece | High | Medium |
| Weld parameter database | Weld database | Store/recall welding parameters | High | Low |
| Weld schedule | Weld schedule | Parameter sets cho different segments | Medium | Low |
| Multi-pass welding | Multi-pass | Automatic multi-pass with offset | Medium | High |
| Crater fill | Crater fill | End crater compensation | Medium | Low |
| Burn-back time | Burn-back | Wire retraction timing | Low | Low |
| Gas pre/post flow | Gas timing | Shield gas timing control | Low | Low |

### 3.6 HMI Features

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| **Position display modes** | POV (Point of View) | Switch Joint/World/Tool/Base coordinates | **Critical** | Low |
| **Jog coordinate system** | Jog frame | Jog trong World/Tool/Base frame | **Critical** | Medium |
| Message system | $MSG_T | User messages/dialogs với buttons | High | Low |
| Variable display | Variable watch | Real-time display của robot variables | High | Low |
| Override control | $OV_PRO | Program speed override (1-100%) | High | Low |
| Trace/Oscilloscope | Trace | Signal recording và display | Medium | Medium |
| User keys | User keys F1-F6 | Customizable soft buttons | Medium | Low |
| Status bar | Status display | Connection, mode, program status | Medium | Low |
| Program pointer | BCO/Run pointer | Hiển thị current line in program | Medium | Low |

### 3.7 Program Execution

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Step mode | Step mode | Execute program line-by-line | High | Medium |
| BCO run | BCO (Block Coincidence) | Move to first point before running | High | Medium |
| Program selection | Program select | Load/select programs | High | Low |
| Start/Stop/Pause | R key, Start/Stop | Program execution control | High | Low |
| Backward execution | Backward | Run program in reverse | Medium | High |
| Reset program | Reset | Return to start of program | Medium | Low |

### 3.8 Diagnostics

| Feature | KUKA Name | Description | Priority | Complexity |
|---------|-----------|-------------|----------|------------|
| Message log | Message log | System messages với timestamp | High | Low |
| Motor temperature | Motor diagnostics | Monitor motor temperatures | Medium | Low |
| Cycle time | Cycle time display | Display program cycle time | Medium | Low |
| I/O monitor | I/O display | Real-time I/O states | Medium | Low |
| Brake test | Brake test | Test motor brakes | Low | Medium |
| Mastering check | Mastering check | Verify robot mastering | Low | Low |

---

## 4. Priority Matrix - Recommended Implementation Order

### Phase 7: Essential HMI Improvements (Next)

| Priority | Feature | Why Important |
|----------|---------|---------------|
| 1 | Position display modes (Joint/World/Tool/Base) | Fundamental cho operation |
| 2 | Jog coordinate system selection | Essential cho teaching |
| 3 | Point teaching (Touch-up) | Core workflow cho programming |
| 4 | Override control | Speed adjustment |
| 5 | Digital I/O display | Debugging và monitoring |

### Phase 8: Programming Features

| Priority | Feature | Why Important |
|----------|---------|---------------|
| 1 | Inline forms for motion | Simplified programming |
| 2 | Point list management | Teaching workflow |
| 3 | Program structure | Code organization |
| 4 | Step execution | Debugging |
| 5 | BCO run | Safe program start |

### Phase 9: Advanced Motion

| Priority | Feature | Why Important |
|----------|---------|---------------|
| 1 | Approximate positioning ($APO) | Smooth motion |
| 2 | Constant path velocity | Welding quality |
| 3 | Spline motion | Advanced paths |

### Phase 10: Welding Enhancement

| Priority | Feature | Why Important |
|----------|---------|---------------|
| 1 | Weld parameter database | Repeatability |
| 2 | Touch sensing | Workpiece detection |
| 3 | Arc sensing (TAST) | Seam tracking |

---

## 4. UI Structure Overview

```
MainWindow
├── Header Bar
│   ├── Connection Status
│   ├── Robot State
│   ├── Mode Selector ←─────────── KUKA T1/T2/AUT/EXT
│   └── E-Stop Button
│
├── Navigation (Left)
│   ├── Motion
│   ├── Program
│   ├── I/O
│   ├── Configuration
│   └── Diagnostics
│
├── Content Area
│   ├── Motion Page
│   │   ├── Jog Panel ←─────────── KUKA SmartPAD jog
│   │   ├── Position Display ←──── KUKA position display
│   │   └── 3D Viewport
│   │
│   └── Configuration Page
│       ├── Connection
│       ├── Robot Catalog ←─────── KUKA WorkVisual
│       ├── Homing ←────────────── KUKA Mastering
│       ├── Tools ←─────────────── KUKA Tool calibration
│       ├── Base Frames ←───────── KUKA Base calibration
│       ├── Robot Settings
│       ├── Safety
│       └── Welding
│
└── Status Bar
    ├── Position (Joint/Cartesian)
    ├── Velocity
    └── Program Status
```

---

## 5. Research Notes

### 5.1 KUKA Documentation References

- KUKA System Software (KSS) 8.x Operating and Programming
- KUKA.ArcTech Basic/Advanced
- KUKA.SafeOperation
- KUKA.WorkVisual User Guide

### 5.2 Key Differences from KUKA

| Aspect | KUKA | Our Implementation |
|--------|------|-------------------|
| Language | KRL (proprietary) | C++/C# |
| Real-time | VxWorks RTOS | Windows + Teensy firmware |
| Safety | Dual-channel redundant | Software + firmware checks |
| HMI | SmartPAD (dedicated) | WPF application |
| Protocol | ProfiSafe, EtherCAT | ZeroMQ, Serial |

---

## 6. Next Steps

1. **Research Phase**: Web research các features KUKA còn thiếu
2. **Priority Assessment**: Đánh giá priority dựa trên use case (welding robot)
3. **Implementation Planning**: Tạo IMPL plans cho features mới
4. **Development**: Implement theo priority order

---

## 7. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-04 | Claude | Initial version - documented P6 features |
| 1.1 | 2026-02-04 | Claude | Added comprehensive future features list with priority matrix |
| 1.2 | 2026-02-04 | Claude | Added Phase 8: Virtual Simulation design (KUKA.OfficeLite-inspired) |
