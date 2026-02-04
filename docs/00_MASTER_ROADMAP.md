# MASTER ROADMAP: 6-DOF ROBOT CONTROLLER

## Document Info
| Item | Value |
|------|-------|
| **Project** | Commercial 6-DOF Robot Controller |
| **Architecture** | PC-Based Standalone |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. VISION & OBJECTIVES

### 1.1. Project Vision
Xây dựng bộ điều khiển robot công nghiệp 6-DOF thương mại, hoàn toàn tự phát triển từ UI đến Firmware, không phụ thuộc vào middleware bên thứ ba (ROS, Gazebo, etc.).

### 1.2. Core Objectives
| ID | Objective | Priority |
|----|-----------|----------|
| OBJ-01 | Motion control chính xác cho robot 6 trục | P0 |
| OBJ-02 | Safety system tuân thủ ISO 10218-1 | P0 |
| OBJ-03 | Hỗ trợ đa chế độ ứng dụng (Welding, Pick&Place, Scan-to-Path) | P1 |
| OBJ-04 | HMI thân thiện, chuyên nghiệp (KUKA-style) | P1 |
| OBJ-05 | Simulation Mode để test offline | P1 |
| OBJ-06 | Vision integration cho seam tracking | P2 |

### 1.3. Success Criteria
- [ ] Robot di chuyển chính xác theo quỹ đạo lập trình
- [ ] Cycle time đáp ứng yêu cầu công nghiệp (< 1ms control loop)
- [ ] Zero safety incidents trong vận hành
- [ ] Hàn được mối hàn chất lượng với seam tracking
- [ ] Operator có thể vận hành mà không cần training chuyên sâu

---

## 2. SYSTEM ARCHITECTURE

### 2.1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    PC-BASED STANDALONE ARCHITECTURE              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  LAYER 1: USER INTERFACE (C# WPF)                          │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │ │
│  │  │   HMI    │ │ 3D View  │ │ Program  │ │  Config  │      │ │
│  │  │  Panel   │ │  Helix   │ │  Editor  │ │  Manager │      │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                    ZeroMQ / Shared Memory                        │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  LAYER 2: CORE LOGIC (C++ 17/20)                           │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │ │
│  │  │  State   │ │ Motion   │ │ Welding  │ │  Vision  │      │ │
│  │  │ Manager  │ │ Planner  │ │Sequencer │ │ Pipeline │      │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐                   │ │
│  │  │Kinematics│ │  Ruckig  │ │  Safety  │                   │ │
│  │  │  IK/FK   │ │   OTG    │ │ Interlock│                   │ │
│  │  └──────────┘ └──────────┘ └──────────┘                   │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                      Serial / USB                                │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  LAYER 3: FIRMWARE (C/C++ - Teensy 4.1)                    │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │ │
│  │  │ grblHAL  │ │ Step/Dir │ │ Encoder  │ │ Safety   │      │ │
│  │  │  Core    │ │Generator │ │ Feedback │ │   I/O    │      │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  EXTERNAL HARDWARE                                          │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │ │
│  │  │  Servo   │ │  Laser   │ │ Welding  │ │  I/O     │      │ │
│  │  │ Drivers  │ │ Profiler │ │  Source  │ │ Modules  │      │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Architecture Principles

| Principle | Description |
|-----------|-------------|
| **Layered** | UI / Core / Firmware tách biệt rõ ràng |
| **Modular** | Mỗi module có interface rõ ràng, có thể test độc lập |
| **Real-time** | Core logic chạy deterministic, không phụ thuộc UI |
| **Safety-first** | Safety logic độc lập, không bị ảnh hưởng bởi lỗi phần mềm khác |
| **Extensible** | Dễ dàng thêm mode mới (Pick&Place, Scan-to-Path) |

### 2.3. Constraints (Ràng buộc)

| Constraint | Reason |
|------------|--------|
| ❌ Không dùng ROS/ROS2 | Standalone, không phụ thuộc middleware |
| ❌ Không dùng Gazebo | Tự xây dựng Simulation Mode |
| ❌ Không dùng distributed system | Single PC deployment |
| ✅ Windows 10/11 | Target platform |
| ✅ C# .NET 6+ | UI layer |
| ✅ C++ 17/20 | Core logic |
| ✅ Teensy 4.1 + grblHAL | Firmware |

---

## 3. APPLICATION MODES

### 3.1. Mode Overview

```
                    ┌─────────────────────┐
                    │    CORE PLATFORM    │
                    │  (Motion + Safety)  │
                    └──────────┬──────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   ┌─────────────┐      ┌─────────────┐      ┌─────────────┐
   │   MODE 1    │      │   MODE 2    │      │   MODE 3    │
   │   WELDING   │      │ PICK&PLACE  │      │ SCAN-TO-PATH│
   │             │      │             │      │             │
   │ - Arc Weld  │      │ - Gripper   │      │ - 3D Scan   │
   │ - Seam Track│      │ - Vision    │      │ - Path Gen  │
   │ - Weaving   │      │ - Palletize │      │ - Offline   │
   └─────────────┘      └─────────────┘      └─────────────┘
        P0                   P2                   P2
   (Primary Mode)      (Future Mode)        (Future Mode)
```

### 3.2. Mode Details

#### MODE 1: WELDING (Primary - P0)
| Aspect | Details |
|--------|---------|
| **Description** | Hàn MIG/MAG với điều khiển trình tự chính xác |
| **Features** | Arc control, Pre-flow/Post-flow, Crater fill, Burnback, Weaving |
| **Hardware** | Nguồn hàn MIG/MAG, Laser profiler (optional) |
| **Tech Stack** | WeldingSequencer (C++), I/O interface, Analog 0-10V |
| **Reference Docs** | Thiết Kế Module Điều Khiển Hàn MIG_MAG.md |

#### MODE 2: PICK & PLACE (Future - P2)
| Aspect | Details |
|--------|---------|
| **Description** | Gắp và đặt vật với gripper |
| **Features** | Gripper control, Vision guidance, Palletizing |
| **Hardware** | Gripper (pneumatic/electric), Camera |
| **Tech Stack** | GripperController, VisionPipeline |
| **Reference Docs** | Robot Welding sang Pick & Place.md |

#### MODE 3: SCAN-TO-PATH (Future - P2)
| Aspect | Details |
|--------|---------|
| **Description** | Quét 3D và tự động tạo đường chạy |
| **Features** | 3D scanning, Point cloud processing, Path generation |
| **Hardware** | 3D Laser scanner |
| **Tech Stack** | Open3D/PCL, PathGenerator |
| **Reference Docs** | Robot Hàn Scan-to-Path Tái tạo 3D.md |

---

## 4. TECH STACK

### 4.1. Layer 1: UI (C# WPF)

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Framework | .NET | 6.0+ | Runtime |
| UI Framework | WPF | - | Desktop UI |
| 3D Engine | Helix Toolkit | Latest | 3D visualization |
| 3D (High Perf) | Helix Toolkit SharpDX | Latest | DirectX rendering |
| MVVM | CommunityToolkit.Mvvm | Latest | UI pattern |
| IPC Client | NetMQ (ZeroMQ) | Latest | Communication |
| Logging | Serilog | Latest | Structured logging |
| Config | System.Text.Json | Built-in | JSON config |

### 4.2. Layer 2: Core Logic (C++)

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Compiler | MSVC | 2022+ | C++17/20 |
| Build | CMake | 3.20+ | Build system |
| Kinematics | Robotics Library (RL) | Latest | IK/FK solver |
| Trajectory | Ruckig | 0.9+ | OTG jerk-limited |
| Math | Eigen | 3.4+ | Linear algebra |
| IPC Server | cppzmq (ZeroMQ) | Latest | Communication |
| Point Cloud | Open3D / PCL | Latest | Vision processing |
| Logging | spdlog | Latest | Fast logging |
| JSON | nlohmann/json | Latest | Config parsing |
| State Machine | Custom (std::variant) | - | FSM implementation |

### 4.3. Layer 3: Firmware (C/C++)

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Platform | Teensy 4.1 | - | MCU (600MHz ARM) |
| Motion Core | grblHAL | Latest | Step generation |
| Protocol | G-code | - | Motion commands |
| Communication | USB Serial | - | PC connection |

### 4.4. External Libraries Summary

```
┌─────────────────────────────────────────────────────────────┐
│                    DEPENDENCY GRAPH                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  C# UI Layer:                                                │
│  ├── Helix Toolkit ──► SharpDX (optional)                   │
│  ├── NetMQ ──► libzmq                                       │
│  └── CommunityToolkit.Mvvm                                  │
│                                                              │
│  C++ Core Layer:                                             │
│  ├── Robotics Library (RL) ──► Eigen, libxml2               │
│  ├── Ruckig (header-only)                                   │
│  ├── Open3D ──► Eigen, TBB, fmt                             │
│  ├── cppzmq ──► libzmq                                      │
│  └── spdlog (header-only)                                   │
│                                                              │
│  Firmware:                                                   │
│  └── grblHAL ──► Teensy Core                                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. DEVELOPMENT PHASES

### 5.1. Phase Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     DEVELOPMENT PHASES                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  PHASE 1          PHASE 2          PHASE 3          PHASE 4     │
│  FOUNDATION       MOTION           WELDING          VISION      │
│                                                                  │
│  ┌─────────┐     ┌─────────┐      ┌─────────┐     ┌─────────┐  │
│  │ Project │     │  State  │      │ Welding │     │  Laser  │  │
│  │ Setup   │     │ Machine │      │Sequencer│     │ Sensor  │  │
│  ├─────────┤     ├─────────┤      ├─────────┤     ├─────────┤  │
│  │  IPC    │     │Kinematics│     │  Arc    │     │  Seam   │  │
│  │ Layer   │     │  IK/FK  │      │ Control │     │Detection│  │
│  ├─────────┤     ├─────────┤      ├─────────┤     ├─────────┤  │
│  │ Config  │     │ Ruckig  │      │ Weaving │     │ Tracking│  │
│  │ System  │     │  OTG    │      │Patterns │     │ Loop    │  │
│  ├─────────┤     ├─────────┤      ├─────────┤     ├─────────┤  │
│  │  HMI    │     │ grblHAL │      │Weld UI  │     │ Vision  │  │
│  │ Shell   │     │ Firmware│      │ Editor  │     │   UI    │  │
│  ├─────────┤     ├─────────┤      ├─────────┤     ├─────────┤  │
│  │Logging  │     │Simulation│     │  Test   │     │Hand-Eye │  │
│  │Framework│     │  Mode   │      │  Run    │     │ Calib   │  │
│  └─────────┘     └─────────┘      └─────────┘     └─────────┘  │
│                                                                  │
│  ════════════════════════════════════════════════════════════   │
│       │               │                │               │        │
│       ▼               ▼                ▼               ▼        │
│    Milestone 1    Milestone 2      Milestone 3    Milestone 4   │
│   "Hello Robot"  "Robot Moves"   "Robot Welds"  "Smart Weld"   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2. Phase Details

#### PHASE 1: FOUNDATION
| ID | Task | Description | Dependencies |
|----|------|-------------|--------------|
| P1-01 | Project Structure | CMake + .NET solution setup | None |
| P1-02 | IPC Layer | ZeroMQ communication C# ↔ C++ | P1-01 |
| P1-03 | Config System | YAML/JSON config loader | P1-01 |
| P1-04 | Logging Framework | spdlog (C++) + Serilog (C#) | P1-01 |
| P1-05 | HMI Shell | Basic WPF window, navigation | P1-01 |
| P1-06 | Robot Model Loader | Load URDF/custom format | P1-01 |

**Milestone 1: "Hello Robot"**
- [ ] C# UI khởi động, hiển thị mô hình robot 3D
- [ ] C++ Core chạy, giao tiếp được với UI
- [ ] Config load được từ file

#### PHASE 2: MOTION CORE
| ID | Task | Description | Dependencies |
|----|------|-------------|--------------|
| P2-01 | SystemStateManager | FSM theo ISO 10218-1 | P1-02 |
| P2-02 | Safety Interlocks | E-Stop, Deadman, Limits | P2-01 |
| P2-03 | Kinematics Module | IK/FK với Robotics Library | P1-06 |
| P2-04 | Trajectory Generator | Ruckig OTG integration | P2-03 |
| P2-05 | grblHAL Integration | Firmware communication | P2-04 |
| P2-06 | Jog Mode | Manual jogging (Joint/Cartesian) | P2-03, P2-01 |
| P2-07 | Simulation Mode | VirtualController | P2-04 |
| P2-08 | 3D Visualization | Robot animation, Ghost Robot | P2-03, P1-05 |

**Milestone 2: "Robot Moves"**
- [ ] Robot Jog được bằng tay (Joint mode + Cartesian mode)
- [ ] State Machine hoạt động đúng (Idle → Ready → Run)
- [ ] E-Stop hoạt động
- [ ] Simulation Mode chạy được offline

#### PHASE 3: WELDING MODE
| ID | Task | Description | Dependencies |
|----|------|-------------|--------------|
| P3-01 | WeldingSequencer | Welding FSM | P2-01 |
| P3-02 | I/O Interface | Digital/Analog I/O mapping | P3-01 |
| P3-03 | Arc Control | Pre-flow, Ignition, Burnback | P3-02 |
| P3-04 | Weaving Patterns | Zigzag, Sin, Triangle | P3-01, P2-04 |
| P3-05 | Welding Program | Program structure, teach | P3-01 |
| P3-06 | Weld UI | Program editor, parameter | P3-05, P1-05 |
| P3-07 | Trail Rendering | Weld path visualization | P2-08 |

**Milestone 3: "Robot Welds"**
- [ ] Robot thực hiện được chu trình hàn hoàn chỉnh
- [ ] Pre-flow → Arc → Weld → Crater → Post-flow
- [ ] Weaving hoạt động
- [ ] UI hiển thị được đường hàn

#### PHASE 4: VISION INTEGRATION
| ID | Task | Description | Dependencies |
|----|------|-------------|--------------|
| P4-01 | Laser Profiler Driver | Hikrobot/Mech-Mind SDK | P1-02 |
| P4-02 | Profile Processing | Point cloud acquisition | P4-01 |
| P4-03 | Seam Detection | RANSAC, Steger algorithm | P4-02 |
| P4-04 | Hand-Eye Calibration | AX=XB solver | P2-03, P4-01 |
| P4-05 | Tracking Loop | Real-time seam tracking | P4-03, P3-01 |
| P4-06 | Vision UI | Calibration, monitoring | P4-01, P1-05 |

**Milestone 4: "Smart Weld"**
- [ ] Laser profiler đọc được profile
- [ ] Phát hiện được mối hàn tự động
- [ ] Seam tracking real-time hoạt động
- [ ] Bù trừ được sai lệch vị trí phôi

---

## 6. MODULE DEPENDENCIES

### 6.1. Dependency Matrix

```
┌─────────────────────────────────────────────────────────────────┐
│                    MODULE DEPENDENCIES                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│                      ┌──────────────┐                           │
│                      │   Config     │                           │
│                      │   System     │                           │
│                      └──────┬───────┘                           │
│                             │                                    │
│              ┌──────────────┼──────────────┐                    │
│              │              │              │                    │
│              ▼              ▼              ▼                    │
│       ┌──────────┐   ┌──────────┐   ┌──────────┐               │
│       │  Logger  │   │   IPC    │   │  Robot   │               │
│       │          │   │  Layer   │   │  Model   │               │
│       └──────────┘   └────┬─────┘   └────┬─────┘               │
│                           │              │                      │
│                           │         ┌────┴─────┐               │
│                           │         │Kinematics│               │
│                           │         │  IK/FK   │               │
│                           │         └────┬─────┘               │
│                           │              │                      │
│              ┌────────────┴──────────────┤                      │
│              │                           │                      │
│              ▼                           ▼                      │
│       ┌──────────┐                ┌──────────┐                 │
│       │  State   │                │ Ruckig   │                 │
│       │ Manager  │◄───────────────│   OTG    │                 │
│       └────┬─────┘                └────┬─────┘                 │
│            │                           │                        │
│       ┌────┴────────────┬──────────────┤                       │
│       │                 │              │                        │
│       ▼                 ▼              ▼                        │
│  ┌─────────┐     ┌──────────┐   ┌──────────┐                   │
│  │ Welding │     │   Jog    │   │ grblHAL  │                   │
│  │Sequencer│     │  Mode    │   │ Driver   │                   │
│  └────┬────┘     └──────────┘   └──────────┘                   │
│       │                                                         │
│       ▼                                                         │
│  ┌─────────┐                                                    │
│  │ Vision  │                                                    │
│  │Pipeline │                                                    │
│  └─────────┘                                                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2. Build Order

Thứ tự build đảm bảo dependencies:

```
1. Config System
2. Logger
3. IPC Layer
4. Robot Model
5. Kinematics (IK/FK)
6. State Manager
7. Ruckig OTG
8. grblHAL Driver
9. Jog Mode
10. Welding Sequencer
11. Vision Pipeline
```

---

## 7. DOCUMENTATION STRUCTURE

### 7.1. Folder Structure

```
📁 Robot_controller/
├── 📁 docs/
│   ├── 📄 00_MASTER_ROADMAP.md              ← This file
│   ├── 📄 01_ARCHITECTURE_OVERVIEW.md       ← System architecture
│   │
│   ├── 📁 core_platform/                    ← Core module specs
│   │   ├── 📄 CORE_01_Project_Setup.md
│   │   ├── 📄 CORE_02_IPC_Layer.md
│   │   ├── 📄 CORE_03_Config_System.md
│   │   ├── 📄 CORE_04_State_Machine.md
│   │   ├── 📄 CORE_05_Kinematics.md
│   │   ├── 📄 CORE_06_Trajectory.md
│   │   ├── 📄 CORE_07_grblHAL.md
│   │   ├── 📄 CORE_08_HMI_Framework.md
│   │   └── 📄 CORE_09_Simulation.md
│   │
│   ├── 📁 modes/                            ← Application modes
│   │   ├── 📄 MODE_01_Welding.md
│   │   ├── 📄 MODE_02_PickPlace.md
│   │   └── 📄 MODE_03_ScanToPath.md
│   │
│   ├── 📁 phases/                           ← Phase details
│   │   ├── 📄 PHASE_1_Foundation.md
│   │   ├── 📄 PHASE_2_Motion.md
│   │   ├── 📄 PHASE_3_Welding.md
│   │   └── 📄 PHASE_4_Vision.md
│   │
│   └── 📁 testing/                          ← Test plans & checklists
│       ├── 📄 TEST_PHASE_1_Foundation.md
│       ├── 📄 TEST_PHASE_2_Motion.md
│       ├── 📄 TEST_PHASE_3_Welding.md
│       ├── 📄 TEST_PHASE_4_Vision.md
│       └── 📄 COMMISSIONING_CHECKLIST.md
│
├── 📁 ressearch_doc_md/                     ← Research documents
│   └── (15 markdown files)
│
├── 📁 src/                                  ← Source code
│   ├── 📁 ui/                               ← C# WPF
│   ├── 📁 core/                             ← C++ Core
│   └── 📁 firmware/                         ← Teensy/grblHAL
│
└── 📁 config/                               ← Configuration files
```

### 7.2. Document Template

Mỗi module document nên có cấu trúc:

```markdown
# MODULE: [Module Name]

## 1. Overview
- Purpose
- Scope
- Dependencies

## 2. Requirements
- Functional requirements
- Non-functional requirements

## 3. Design
- Architecture
- Interfaces
- Data structures

## 4. Implementation
- Tech stack
- Key algorithms
- Code structure

## 5. Testing
- Unit tests
- Integration tests
- Acceptance criteria

## 6. References
- Research docs
- External resources
```

---

## 8. RISK MANAGEMENT

### 8.1. Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| grblHAL không đáp ứng 6-DOF | High | Medium | Tùy chỉnh firmware, fallback to custom step generation |
| Real-time performance trên Windows | High | Medium | Sử dụng high-priority threads, separate control loop |
| Kinematics singularity | Medium | Medium | Singularity avoidance trong trajectory planning |
| Vision latency quá cao | Medium | Low | Optimize pipeline, use hardware acceleration |
| IPC bottleneck | Medium | Low | Benchmark early, use shared memory if needed |

### 8.2. Mitigation Strategies

1. **Prototype Early**: Xây dựng vertical slice (UI→Core→Firmware) sớm để validate architecture
2. **Benchmark Continuously**: Đo cycle time, latency ở mỗi milestone
3. **Fallback Plans**: Có backup solution cho các critical components
4. **Incremental Integration**: Integrate từng module, test liên tục

---

## 9. QUALITY ASSURANCE

### 9.1. Testing Strategy

| Level | Scope | Tools |
|-------|-------|-------|
| Unit Tests | Individual functions/classes | Google Test (C++), xUnit (C#) |
| Integration Tests | Module interactions | Custom test harness |
| System Tests | End-to-end workflows | Simulation Mode |
| Hardware Tests | Real robot | Manual test procedures |

### 9.2. Code Quality

| Aspect | Standard |
|--------|----------|
| C++ Style | Google C++ Style Guide |
| C# Style | Microsoft C# Conventions |
| Documentation | Doxygen (C++), XML Comments (C#) |
| Version Control | Git, feature branches |
| Code Review | Required for all merges |

---

## 10. NEXT STEPS

### Immediate Actions

| Priority | Action | Owner |
|----------|--------|-------|
| 1 | Tạo Phase 1 detail document | - |
| 2 | Setup project structure (CMake + .NET) | - |
| 3 | Implement IPC layer prototype | - |
| 4 | Evaluate Robotics Library với robot model | - |

### Document Backlog

- [ ] PHASE_1_Foundation.md (Chi tiết tasks Phase 1)
- [ ] CORE_01_Project_Setup.md (Build system, folder structure)
- [ ] CORE_02_IPC_Layer.md (ZeroMQ design)
- [ ] MODE_01_Welding.md (Welding mode specification)

---

## APPENDIX

### A. Reference Documents

| Document | Location | Content |
|----------|----------|---------|
| PROJECT BLUEPRINT | ressearch_doc_md/ | Overall architecture |
| FSM Design | ressearch_doc_md/Thiết Kế FSM Robot... | State machine |
| Welding Module | ressearch_doc_md/Thiết Kế Module Điều Khiển Hàn... | Welding logic |
| Simulation Mode | ressearch_doc_md/Thiết Kế Mô Phỏng Robot... | VirtualController |
| Kinematics | ressearch_doc_md/Robotics Library... | IK/FK |
| Trajectory | ressearch_doc_md/Tích hợp Ruckig... | OTG |
| grblHAL | ressearch_doc_md/Tối ưu grblHAL... | Firmware |
| Vision | ressearch_doc_md/Robot Hàn Cảm Biến Laser... | Laser sensor |
| HMI | ressearch_doc_md/Thiết kế HMI Robot KUKA... | UI design |

### B. Glossary

| Term | Definition |
|------|------------|
| **FSM** | Finite State Machine - Máy trạng thái hữu hạn |
| **IK** | Inverse Kinematics - Động học nghịch |
| **FK** | Forward Kinematics - Động học thuận |
| **OTG** | Online Trajectory Generation |
| **TCP** | Tool Center Point - Điểm trung tâm dụng cụ |
| **WCS** | Work Coordinate System - Hệ tọa độ làm việc |
| **IPC** | Inter-Process Communication |
| **HMI** | Human-Machine Interface |

### C. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
