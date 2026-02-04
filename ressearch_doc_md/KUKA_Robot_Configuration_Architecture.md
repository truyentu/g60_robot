# KUKA Robot Configuration Architecture

## Overview

Tài liệu này tổng hợp kiến trúc cấu hình robot của KUKA, bao gồm cách KUKA quản lý nhiều loại robot khác nhau với các thông số kinematic (DH parameters) khác nhau. Thông tin được thu thập từ các nguồn chính thức của KUKA bao gồm WorkVisual documentation, Machine Data documentation, và KRC controller manuals.

**Mục đích:** Làm tài liệu tham khảo để thiết kế hệ thống Multi-Robot Configuration cho project Robot Controller.

---

## 1. Tổng quan Kiến trúc KUKA

### 1.1 Các thành phần chính

| Component | Mô tả |
|-----------|-------|
| **KRC Controller** | Robot controller (KR C4, KR C5) - phần cứng điều khiển |
| **KUKA System Software (KSS)** | Hệ điều hành thời gian thực trên controller |
| **WorkVisual** | Engineering software để cấu hình, lập trình offline |
| **smartPAD** | Teach pendant - giao diện vận hành |
| **Robot Catalog** | Thư viện các model robot với thông số kinematic |

### 1.2 Phân tách Configuration

KUKA phân tách rõ ràng giữa:

1. **Robot Model Definition** (trong Catalog) - Định nghĩa chuẩn của một dòng robot
2. **Robot Instance Configuration** - Cấu hình cụ thể của một robot đã deploy
3. **Controller Configuration** - Cấu hình chung của controller

```
┌─────────────────────────────────────────────────────────────────┐
│                        WorkVisual                                │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   Robot Catalogs                         │    │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐        │    │
│  │  │KR 6 R900│ │KR 16    │ │KR 120   │ │Custom   │        │    │
│  │  │         │ │R2010    │ │R2500    │ │Robot    │        │    │
│  │  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘        │    │
│  │       │           │           │           │              │    │
│  │       └───────────┴─────┬─────┴───────────┘              │    │
│  │                         │                                 │    │
│  │                    Select Model                           │    │
│  │                         │                                 │    │
│  └─────────────────────────┼─────────────────────────────────┘    │
│                            ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Robot Instance (R1, R2, ...)                │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │    │
│  │  │$MACHINE.DAT │  │$ROBCOR.DAT  │  │$CONFIG.DAT  │      │    │
│  │  │(Hardware)   │  │(Kinematics) │  │(Settings)   │      │    │
│  │  └─────────────┘  └─────────────┘  └─────────────┘      │    │
│  └─────────────────────────────────────────────────────────┘    │
│                            │                                      │
│                       Deploy to                                   │
│                            ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                  KRC Controller                          │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Cấu trúc File System của KUKA

### 2.1 Directory Structure trên KRC Controller

```
KRC:\
├── R1\                              # Robot instance 1
│   ├── MADA\                        # Machine Data (robot-specific)
│   │   ├── $MACHINE.DAT             # Hardware configuration
│   │   ├── $ROBCOR.DAT              # Robot correction/kinematic data
│   │   ├── MACHINE.UPG              # Upgrade file
│   │   └── ROBCOR.UPG               # Upgrade file
│   ├── SYSTEM\                      # System files
│   │   ├── $CONFIG.DAT              # General configuration
│   │   ├── BAS.SRC                  # Basic motion package
│   │   ├── IR_STOPM.SRC             # Fault handling
│   │   └── SPS.SUB                  # Submit file (parallel monitoring)
│   ├── TP\                          # Technology packages
│   │   └── P00.DAT/SRC              # PLC coupling
│   └── Program files...
│
├── R2\                              # Robot instance 2 (nếu có)
│   └── [same structure as R1]
│
├── STEU\                            # Controller-level settings
│   └── MADA\
│       └── $CUSTOM.DAT              # Custom I/O configuration
│
└── [System files]
```

### 2.2 Ý nghĩa các thư mục

| Path | Mục đích |
|------|----------|
| `R1/MADA/` | Machine Data - Dữ liệu phần cứng và kinematic của robot |
| `R1/SYSTEM/` | System files - Các package cơ bản cho motion control |
| `R1/TP/` | Technology Packages - Các module tùy chọn (welding, etc.) |
| `STEU/MADA/` | Controller-level configuration |

---

## 3. File $MACHINE.DAT - Hardware Configuration

### 3.1 Mục đích

File `$MACHINE.DAT` chứa tất cả dữ liệu cấu hình phần cứng cần thiết để vận hành robot. Bao gồm thông số của drives, motors, và axis kinematic systems.

### 3.2 Các nhóm tham số chính

#### 3.2.1 Axis Configuration

```krl
; Axis limits
$AXIS_HOME[1] = 0.0          ; Home position
$AXIS_DIR[1] = 1             ; Direction (+1 or -1)
$SOFTP_END[1] = 185.0        ; Positive soft limit (degrees)
$SOFTN_END[1] = -185.0       ; Negative soft limit (degrees)
```

#### 3.2.2 Velocity & Acceleration

```krl
; Velocity limits
$VEL_AXIS[1] = 156.0         ; Max velocity (deg/s)
$VEL_MA[1] = 100             ; Velocity percentage

; Acceleration parameters
$ACC_AXIS[1] = 100           ; Acceleration (%)
$RAISE_TIME[1] = 500         ; Ramp-up time (ms)
$RAISE_T_MOT[1] = 300        ; Motor acceleration time (ms)
```

#### 3.2.3 Motor & Drive Data

```krl
; Motor configuration
$MOT_TYPE[1] = #SERVO        ; Motor type
$GEAR[1] = -101              ; Gear ratio (negative = direction)
$RATED_CURRENT[1] = 12.5     ; Rated current (A)
$PEAK_CURRENT[1] = 37.5      ; Peak current (A)
```

#### 3.2.4 Monitoring Parameters

```krl
; Safety monitoring
$ACC_CAR_STOP = TRUE         ; Enable Cartesian acceleration monitoring
$TORQUE_AXIS[1] = 100        ; Torque limit (%)
$CURR_LIM[1] = 100           ; Current limit (%)
```

### 3.3 Danh sách tham số đầy đủ

| Variable | Type | Unit | Description |
|----------|------|------|-------------|
| `$SOFTP_END[n]` | REAL | deg | Positive software limit |
| `$SOFTN_END[n]` | REAL | deg | Negative software limit |
| `$VEL_AXIS[n]` | REAL | deg/s | Maximum axis velocity |
| `$ACC_AXIS[n]` | INT | % | Axis acceleration |
| `$RAISE_TIME[n]` | INT | ms | Acceleration ramp time |
| `$RAISE_T_MOT[n]` | INT | ms | Motor acceleration time |
| `$GEAR[n]` | INT | - | Gear ratio (with direction) |
| `$AXIS_HOME[n]` | REAL | deg | Home position |
| `$AXIS_DIR[n]` | INT | - | Axis direction (+1/-1) |
| `$RATED_CURRENT[n]` | REAL | A | Motor rated current |
| `$PEAK_CURRENT[n]` | REAL | A | Motor peak current |
| `$TORQUE_AXIS[n]` | INT | % | Torque monitoring limit |

---

## 4. File $ROBCOR.DAT - Kinematic Model

### 4.1 Mục đích

File `$ROBCOR.DAT` chứa dữ liệu mô hình động học (kinematic model) và các giá trị hiệu chuẩn (calibration) của robot. Đây là file quan trọng nhất để định nghĩa hình học của robot.

### 4.2 DH Parameters (Denavit-Hartenberg)

KUKA sử dụng Modified DH Convention với các tham số:

| Parameter | Symbol | Ý nghĩa |
|-----------|--------|---------|
| **a** | Link Length | Khoảng cách dọc theo trục X (mm) |
| **alpha** | Link Twist | Góc xoay quanh trục X (degrees) |
| **d** | Link Offset | Khoảng cách dọc theo trục Z (mm) |
| **theta** | Joint Angle | Góc quay của khớp (variable cho revolute joint) |

```krl
; DH Parameters structure trong $ROBCOR.DAT
; Axis 1-6 parameters
$DH_AXISNO[1] = 1
$DH_A[1] = 0.0               ; Link length a1 (mm)
$DH_ALPHA[1] = -90.0         ; Link twist alpha1 (deg)
$DH_D[1] = 400.0             ; Link offset d1 (mm)
$DH_THETA[1] = 0.0           ; Theta offset (deg)

$DH_AXISNO[2] = 2
$DH_A[2] = 560.0             ; a2
$DH_ALPHA[2] = 0.0           ; alpha2
$DH_D[2] = 0.0               ; d2
$DH_THETA[2] = -90.0         ; theta offset

; ... continues for all 6 axes
```

### 4.3 Kinematic Coupling

Một số robot có kinematic coupling giữa các khớp (ví dụ: wrist coupling):

```krl
; Wrist coupling parameters
$COUP_COMP[1] = 0.0          ; Coupling compensation axis 4-5
$COUP_COMP[2] = 0.0          ; Coupling compensation axis 5-6
$COUP_KIN = #NONE            ; Coupling type: #NONE, #PARALLEL, etc.
```

### 4.4 Calibration Data

Dữ liệu hiệu chuẩn được lưu để bù sai số cơ khí:

```krl
; Calibration offsets
$CAL_DIFF[1] = 0.0           ; Calibration offset axis 1 (deg)
$CAL_DIFF[2] = 0.0           ; Calibration offset axis 2
$CAL_DIFF[3] = 0.0           ; Calibration offset axis 3
$CAL_DIFF[4] = 0.0           ; Calibration offset axis 4
$CAL_DIFF[5] = 0.0           ; Calibration offset axis 5
$CAL_DIFF[6] = 0.0           ; Calibration offset axis 6
```

### 4.5 Ví dụ DH Parameters cho một số robot KUKA

#### KR 6 R900 (6kg, 901mm reach)

| Joint | a (mm) | alpha (deg) | d (mm) | theta offset |
|-------|--------|-------------|--------|--------------|
| 1 | 25 | -90 | 400 | 0 |
| 2 | 455 | 0 | 0 | -90 |
| 3 | 35 | -90 | 0 | 0 |
| 4 | 0 | 90 | 420 | 0 |
| 5 | 0 | -90 | 0 | 0 |
| 6 | 0 | 0 | 80 | 0 |

#### KR 16 R2010 (16kg, 2010mm reach)

| Joint | a (mm) | alpha (deg) | d (mm) | theta offset |
|-------|--------|-------------|--------|--------------|
| 1 | 160 | -90 | 675 | 0 |
| 2 | 780 | 0 | 0 | -90 |
| 3 | 150 | -90 | 0 | 0 |
| 4 | 0 | 90 | 860 | 0 |
| 5 | 0 | -90 | 0 | 0 |
| 6 | 0 | 0 | 152 | 0 |

---

## 5. File $CONFIG.DAT - General Configuration

### 5.1 Mục đích

Chứa cấu hình chung cho robot instance, bao gồm:

- Tool definitions (TCP)
- Base/World frame definitions
- User frames
- Default motion parameters

### 5.2 Tool Frame Configuration

```krl
; Tool frame definition
$TOOL = {X 0.0, Y 0.0, Z 80.0, A 0.0, B 0.0, C 0.0}
$LOAD = {M 2.5, CM {X 0.0, Y 0.0, Z 50.0}, J {X 0.01, Y 0.01, Z 0.01}}
```

| Variable | Description |
|----------|-------------|
| `$TOOL` | TCP position relative to flange (X,Y,Z in mm; A,B,C in deg) |
| `$LOAD` | Tool mass (M in kg), center of mass (CM), inertia (J) |

### 5.3 Base Frame Configuration

```krl
; Base/World frame
$BASE = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
$WORLD = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
```

### 5.4 User Frames

```krl
; User-defined frames
$NULLFRAME = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
$UFRAME[1] = {X 100.0, Y 200.0, Z 0.0, A 0.0, B 0.0, C 0.0}
```

---

## 6. WorkVisual Robot Catalog

### 6.1 Catalog Structure

WorkVisual sử dụng hệ thống Catalog để quản lý các thành phần:

```
WorkVisual Catalogs
├── KukaControllers           # Controller hardware catalog
│   ├── KR C4
│   ├── KR C5
│   └── VKR C4
│
├── KukaRobots                # Robot kinematic catalog
│   ├── KR AGILUS Series
│   │   ├── KR 6 R700
│   │   ├── KR 6 R900
│   │   └── KR 10 R1100
│   ├── KR CYBERTECH Series
│   │   ├── KR 8 R1620
│   │   ├── KR 16 R2010
│   │   └── KR 22 R1610
│   ├── KR QUANTEC Series
│   │   ├── KR 120 R2500
│   │   ├── KR 150 R3100
│   │   └── KR 210 R2700
│   └── [Other series...]
│
└── ExternalKinematics        # External axes
    ├── Linear Units
    ├── Positioners
    └── Turn-Tilt Tables
```

### 6.2 Robot Model Data trong Catalog

Mỗi robot model trong catalog chứa:

| Data Category | Contents |
|---------------|----------|
| **Kinematic Data** | DH parameters, joint types, link lengths |
| **Mechanical Limits** | Joint angle limits, workspace envelope |
| **Dynamic Data** | Max velocities, accelerations, torques |
| **Payload Data** | Max payload, inertia limits |
| **Mounting Options** | Floor, ceiling, wall mounting |
| **3D Model** | Visual representation for simulation |

### 6.3 Configuration Proposal

WorkVisual có thể tự động đề xuất cấu hình phần cứng dựa trên robot đã chọn:

```
User selects robot: KR 16 R2010
        │
        ▼
WorkVisual proposes:
├── Required controller: KR C4 or KR C5
├── Required power pack: KPP 600
├── Required cables: specific cable sets
├── Axis configuration: 6-axis standard
└── Optional: external axes
```

---

## 7. Multi-Robot Configuration

### 7.1 Một Controller - Nhiều Robot

KUKA KRC controller có thể điều khiển nhiều robot/kinematic systems:

```
KRC Controller
├── R1 (Robot 1) ─────── KR 6 R900
├── R2 (Robot 2) ─────── KR 16 R2010
├── E1 (External 1) ──── Linear Unit
└── E2 (External 2) ──── Positioner
```

### 7.2 RoboTeam - Multi-Robot Coordination

KUKA.RoboTeam cho phép nhiều robot làm việc phối hợp:

```
┌─────────────────────────────────────────────────┐
│              RoboTeam Configuration              │
├─────────────────────────────────────────────────┤
│  ┌─────────┐      ┌─────────┐      ┌─────────┐ │
│  │ Robot 1 │◄────►│ Robot 2 │◄────►│ Robot 3 │ │
│  │ (Master)│      │(Slave 1)│      │(Slave 2)│ │
│  └────┬────┘      └────┬────┘      └────┬────┘ │
│       │                │                │      │
│       └────────────────┼────────────────┘      │
│                        │                        │
│              Coordinated Motion                 │
└─────────────────────────────────────────────────┘
```

### 7.3 Switching Between Robots

Trong KRL, có thể switch context giữa các robot:

```krl
; Switch to robot 1
$ROB_ACTIVE = 1
PTP HOME

; Switch to robot 2
$ROB_ACTIVE = 2
PTP HOME
```

---

## 8. Configuration Workflow

### 8.1 Offline Configuration với WorkVisual

```
Step 1: Create Project
        │
        ▼
Step 2: Select Controller (KR C4/C5)
        │
        ▼
Step 3: Add Robot from Catalog
        │ ┌─────────────────────────────┐
        │ │ Catalog lookup:             │
        │ │ - Load DH parameters        │
        │ │ - Load limits               │
        │ │ - Load default config       │
        │ └─────────────────────────────┘
        ▼
Step 4: Configure Robot Instance
        │ ┌─────────────────────────────┐
        │ │ Customize:                  │
        │ │ - Tool frame (TCP)          │
        │ │ - Base frame                │
        │ │ - I/O mapping               │
        │ │ - Safety zones              │
        │ └─────────────────────────────┘
        ▼
Step 5: Generate Code
        │ ┌─────────────────────────────┐
        │ │ Auto-generate:              │
        │ │ - $MACHINE.DAT              │
        │ │ - $ROBCOR.DAT               │
        │ │ - $CONFIG.DAT               │
        │ └─────────────────────────────┘
        ▼
Step 6: Deploy to Controller
        │
        ▼
Step 7: On-site Calibration (optional)
        │ ┌─────────────────────────────┐
        │ │ Measure & correct:          │
        │ │ - Update $ROBCOR.DAT        │
        │ │ - Save calibration data     │
        │ └─────────────────────────────┘
        ▼
Step 8: Production Ready
```

### 8.2 Online Configuration (trên Controller)

```
smartPAD
    │
    ├── Configuration Menu
    │   ├── Robot Setup
    │   │   ├── Select Robot Type
    │   │   ├── Axis Configuration
    │   │   └── Payload Configuration
    │   │
    │   ├── Tool Setup
    │   │   ├── Define TCP
    │   │   ├── Tool Mass/Inertia
    │   │   └── Save/Load Tools
    │   │
    │   └── Base Setup
    │       ├── Define Base Frame
    │       ├── User Frames
    │       └── World Frame
    │
    └── Calibration Menu
        ├── Master/Reference
        ├── Axis Calibration
        └── TCP Calibration
```

---

## 9. Key Takeaways cho Robot Controller Project

### 9.1 Design Principles từ KUKA

1. **Separation of Concerns**
   - Model Definition (catalog) vs Instance Configuration (deployed)
   - Hardware config vs Kinematic config vs Runtime config

2. **Hierarchical Configuration**
   - Controller level → Robot level → Tool level
   - Allows inheritance và override

3. **Catalog-based Model Management**
   - Centralized robot model definitions
   - Easy to add new robot types
   - Version control for models

4. **Calibration Separation**
   - Factory default (in catalog)
   - Instance-specific calibration (in robcor)

5. **Structured File Format**
   - Consistent naming convention ($VARIABLE)
   - Type-safe (REAL, INT, BOOL)
   - Grouped by function

### 9.2 Recommended Architecture for Our Project

```
src/config/
├── catalog/                         # Robot Model Catalog (read-only)
│   ├── robots.json                  # Index of all robot models
│   ├── kr6_r900/
│   │   ├── kinematic.json           # DH parameters
│   │   ├── limits.json              # Joint/velocity limits
│   │   ├── dynamics.json            # Dynamic parameters
│   │   └── 3d_model.stl             # Visual model
│   ├── kr16_r2010/
│   │   └── [same structure]
│   └── custom_6dof/
│       └── [same structure]
│
├── instances/                       # Deployed Robot Instances
│   └── robot_r1/
│       ├── machine.json             # Hardware config (~$MACHINE.DAT)
│       ├── robcor.json              # Kinematic + calibration (~$ROBCOR.DAT)
│       ├── config.json              # Runtime config (~$CONFIG.DAT)
│       └── tools/
│           ├── tool_1.json          # Tool definitions
│           └── tool_2.json
│
└── controller/
    ├── system.json                  # Controller-level settings
    └── safety.json                  # Safety configuration
```

### 9.3 JSON Schema Recommendations

#### kinematic.json (Catalog)
```json
{
  "$schema": "robot_kinematic_v1.0",
  "model_name": "KR 6 R900",
  "manufacturer": "KUKA",
  "dof": 6,
  "type": "articulated",
  "dh_convention": "modified",
  "dh_parameters": [
    {"joint": 1, "a": 25, "alpha": -90, "d": 400, "theta_offset": 0},
    {"joint": 2, "a": 455, "alpha": 0, "d": 0, "theta_offset": -90},
    {"joint": 3, "a": 35, "alpha": -90, "d": 0, "theta_offset": 0},
    {"joint": 4, "a": 0, "alpha": 90, "d": 420, "theta_offset": 0},
    {"joint": 5, "a": 0, "alpha": -90, "d": 0, "theta_offset": 0},
    {"joint": 6, "a": 0, "alpha": 0, "d": 80, "theta_offset": 0}
  ],
  "coupling": {
    "type": "none",
    "parameters": []
  }
}
```

#### limits.json (Catalog)
```json
{
  "$schema": "robot_limits_v1.0",
  "joint_limits": [
    {"joint": 1, "min_deg": -185, "max_deg": 185},
    {"joint": 2, "min_deg": -140, "max_deg": 35},
    {"joint": 3, "min_deg": -120, "max_deg": 168},
    {"joint": 4, "min_deg": -185, "max_deg": 185},
    {"joint": 5, "min_deg": -120, "max_deg": 120},
    {"joint": 6, "min_deg": -350, "max_deg": 350}
  ],
  "velocity_limits": [
    {"joint": 1, "max_deg_s": 156},
    {"joint": 2, "max_deg_s": 156},
    {"joint": 3, "max_deg_s": 156},
    {"joint": 4, "max_deg_s": 343},
    {"joint": 5, "max_deg_s": 362},
    {"joint": 6, "max_deg_s": 659}
  ],
  "payload": {
    "max_kg": 6,
    "max_reach_mm": 901
  }
}
```

#### robcor.json (Instance - includes calibration)
```json
{
  "$schema": "robot_instance_kinematic_v1.0",
  "base_model": "kr6_r900",
  "instance_id": "robot_r1",
  "calibration_date": "2024-01-15",
  "calibration_offsets": [
    {"joint": 1, "offset_deg": 0.05},
    {"joint": 2, "offset_deg": -0.12},
    {"joint": 3, "offset_deg": 0.08},
    {"joint": 4, "offset_deg": 0.0},
    {"joint": 5, "offset_deg": -0.03},
    {"joint": 6, "offset_deg": 0.15}
  ],
  "dh_corrections": [],
  "coupling_corrections": []
}
```

---

## 10. References

### 10.1 KUKA Documentation

- WorkVisual 4.0/6.0 Manual (KST WorkVisual)
- Machine Data Documentation (Maschinendaten)
- Expert Programming KSS 5.x/8.x
- KR C2/C4/C5 Operating Instructions
- KUKA System Variables Reference 8.x

### 10.2 Online Resources

- KUKA Xpert Portal: https://xpert.kuka.com
- KUKA Customer Portal: https://my.kuka.com
- RoboDK KUKA Integration: https://robodk.com/doc/en/Robots-kuka.html

### 10.3 Standards

- DIN EN ISO 10218-1: Robots and robotic devices — Safety requirements
- ISO 9787: Robots and robotic devices — Coordinate systems and motion nomenclatures

---

## Appendix A: Complete $MACHINE.DAT Variable List

| Variable | Type | Description |
|----------|------|-------------|
| `$SOFTP_END[n]` | REAL | Positive software limit |
| `$SOFTN_END[n]` | REAL | Negative software limit |
| `$VEL_AXIS[n]` | REAL | Max axis velocity |
| `$ACC_AXIS[n]` | INT | Axis acceleration % |
| `$RAISE_TIME[n]` | INT | Acceleration ramp time |
| `$GEAR[n]` | INT | Gear ratio |
| `$AXIS_DIR[n]` | INT | Axis direction |
| `$AXIS_HOME[n]` | REAL | Home position |
| `$RATED_CURRENT[n]` | REAL | Motor rated current |
| `$PEAK_CURRENT[n]` | REAL | Motor peak current |
| `$TORQUE_AXIS[n]` | INT | Torque limit % |
| `$CURR_LIM[n]` | INT | Current limit % |
| `$ACC_CAR_STOP` | BOOL | Cartesian acc. monitoring |
| `$SERVO_TYPE[n]` | ENUM | Servo amplifier type |

## Appendix B: DH Parameter Convention

### Modified DH (Craig's Convention) - KUKA uses this

```
Transform from frame i-1 to frame i:
1. Rotate about Z(i-1) by θi
2. Translate along Z(i-1) by di
3. Translate along X(i) by ai
4. Rotate about X(i) by αi

Transformation matrix:
T = Rz(θ) * Tz(d) * Tx(a) * Rx(α)
```

### Standard DH (Hartenberg & Denavit)

```
Transform from frame i-1 to frame i:
1. Rotate about Z(i-1) by θi
2. Translate along Z(i-1) by di
3. Translate along X(i-1) by ai
4. Rotate about X(i-1) by αi

Transformation matrix:
T = Rz(θ) * Tz(d) * Tx(a) * Rx(α)
```

**Note:** Our project should support both conventions with a flag in kinematic.json.

---

*Document created: 2024*
*Last updated: Based on KUKA KSS 8.x documentation*
