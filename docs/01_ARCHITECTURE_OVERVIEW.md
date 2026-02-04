# ARCHITECTURE OVERVIEW: 6-DOF Robot Controller

## Document Info
| Item | Value |
|------|-------|
| **Project** | Commercial 6-DOF Robot Controller |
| **Architecture** | PC-Based Standalone (Hybrid) |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Executive Summary

### 1.1. Vision
Xây dựng bộ điều khiển robot công nghiệp 6-DOF thương mại "Made in Vietnam", hoàn toàn tự phát triển từ UI đến Firmware, không phụ thuộc middleware bên thứ ba (ROS, Gazebo).

### 1.2. Architecture Philosophy

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                  │
│                    "PC = BRAIN, MCU = MUSCLE"                    │
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                         PC                               │   │
│   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │   │
│   │  │   Heavy     │  │  Complex    │  │   User      │     │   │
│   │  │   Math      │  │   Logic     │  │ Interface   │     │   │
│   │  │  (IK/FK)    │  │   (FSM)     │  │   (HMI)     │     │   │
│   │  └─────────────┘  └─────────────┘  └─────────────┘     │   │
│   └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                         USB Serial                               │
│                              │                                   │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                        MCU                               │   │
│   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │   │
│   │  │  Real-time  │  │    Step     │  │   Safety    │     │   │
│   │  │   G-code    │  │ Generation  │  │    I/O      │     │   │
│   │  │  Execution  │  │  (600MHz)   │  │  Interlock  │     │   │
│   │  └─────────────┘  └─────────────┘  └─────────────┘     │   │
│   └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3. Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **PC-Based** | Powerful CPU for complex math (IK/FK), easy HMI development |
| **No ROS/Gazebo** | Standalone deployment, no middleware dependencies |
| **C++ Core + C# UI** | Performance where needed, productivity for UI |
| **grblHAL Firmware** | Proven motion control, open-source, customizable |
| **ZeroMQ IPC** | Fast, reliable cross-language communication |

---

## 2. System Architecture

### 2.1. Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SYSTEM ARCHITECTURE                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ╔═══════════════════════════════════════════════════════════════════╗  │
│  ║  LAYER 1: USER INTERFACE (C# WPF .NET 8)                          ║  │
│  ║                                                                    ║  │
│  ║  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────┐ ║  │
│  ║  │   HMI    │ │  3D View │ │ Program  │ │  Config  │ │ Welding │ ║  │
│  ║  │  Shell   │ │  (Helix) │ │  Editor  │ │  Panel   │ │  Scope  │ ║  │
│  ║  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └─────────┘ ║  │
│  ║                                                                    ║  │
│  ║  Tech: WPF, MVVM, Helix Toolkit SharpDX, NetMQ, Serilog          ║  │
│  ╚═══════════════════════════════════════════════════════════════════╝  │
│                                    │                                     │
│                          ZeroMQ (TCP/IPC)                                │
│                       REQ-REP, PUB-SUB, PUSH-PULL                        │
│                                    │                                     │
│  ╔═══════════════════════════════════════════════════════════════════╗  │
│  ║  LAYER 2: CORE LOGIC (C++ 17/20)                                  ║  │
│  ║                                                                    ║  │
│  ║  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────┐ ║  │
│  ║  │  State   │ │Kinematics│ │Trajectory│ │  Config  │ │   IPC   │ ║  │
│  ║  │ Manager  │ │  IK/FK   │ │  Ruckig  │ │  System  │ │ Server  │ ║  │
│  ║  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └─────────┘ ║  │
│  ║  ┌──────────┐ ┌──────────┐ ┌──────────┐                          ║  │
│  ║  │ Welding  │ │  Vision  │ │  grblHAL │                          ║  │
│  ║  │Sequencer │ │ Pipeline │ │  Driver  │                          ║  │
│  ║  └──────────┘ └──────────┘ └──────────┘                          ║  │
│  ║                                                                    ║  │
│  ║  Tech: Robotics Library, Ruckig, Eigen, cppzmq, spdlog           ║  │
│  ╚═══════════════════════════════════════════════════════════════════╝  │
│                                    │                                     │
│                           USB Serial                                     │
│                    115200 baud, Character Counting                       │
│                                    │                                     │
│  ╔═══════════════════════════════════════════════════════════════════╗  │
│  ║  LAYER 3: FIRMWARE (C - Teensy 4.1)                               ║  │
│  ║                                                                    ║  │
│  ║  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────┐ ║  │
│  ║  │ grblHAL  │ │  Step    │ │ Encoder  │ │  Safety  │ │  Weld   │ ║  │
│  ║  │   Core   │ │Generator │ │ Feedback │ │   I/O    │ │   I/O   │ ║  │
│  ║  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └─────────┘ ║  │
│  ║                                                                    ║  │
│  ║  Hardware: ARM Cortex-M7 600MHz, 1MB RAM, 6-axis Step/Dir        ║  │
│  ╚═══════════════════════════════════════════════════════════════════╝  │
│                                    │                                     │
│                              Hardware                                    │
│                                    │                                     │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  EXTERNAL HARDWARE                                                │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ │   │
│  │  │  Servo   │ │  Laser   │ │ Welding  │ │ Digital  │ │ Analog │ │   │
│  │  │ Drivers  │ │ Profiler │ │  Source  │ │   I/O    │ │  I/O   │ │   │
│  │  │  (x6)    │ │(HikRobot)│ │ (MIG/MAG)│ │ (24VDC)  │ │ (0-10V)│ │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └────────┘ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2. Layer Responsibilities

| Layer | Responsibility | Cycle Time | Language |
|-------|---------------|------------|----------|
| **UI** | User interaction, visualization, program editing | Event-driven | C# |
| **Core** | Math, logic, trajectory planning, IPC | 10-50ms | C++ |
| **Firmware** | Step generation, encoder reading, safety I/O | 1ms (deterministic) | C |

### 2.3. Design Principles

| Principle | Implementation |
|-----------|----------------|
| **Layered** | UI / Core / Firmware tách biệt rõ ràng |
| **Modular** | Mỗi module có interface rõ ràng, test độc lập |
| **Real-time** | Core logic deterministic, không phụ thuộc UI |
| **Safety-first** | Safety logic trong MCU, không bị ảnh hưởng bởi PC crash |
| **Extensible** | Dễ dàng thêm mode mới (Welding, Pick&Place, Scan-to-Path) |

---

## 3. Component Architecture

### 3.1. Core Components Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       CORE COMPONENTS DIAGRAM                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                     Configuration System                         │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │    │
│  │  │   robot_    │  │    app_     │  │   user_     │              │    │
│  │  │ config.yaml │  │ config.yaml │  │ config.yaml │              │    │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │    │
│  │         └────────────────┼────────────────┘                      │    │
│  │                          ▼                                       │    │
│  │                  ┌───────────────┐                               │    │
│  │                  │ ConfigManager │ (Singleton)                   │    │
│  │                  └───────┬───────┘                               │    │
│  └──────────────────────────┼───────────────────────────────────────┘    │
│                             │                                            │
│         ┌───────────────────┼───────────────────┐                       │
│         │                   │                   │                       │
│         ▼                   ▼                   ▼                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │ Kinematics  │    │   State     │    │    IPC      │                 │
│  │   Module    │    │  Manager    │    │   Server    │                 │
│  │             │    │             │    │             │                 │
│  │ • DH Params │    │ • FSM       │    │ • REQ-REP   │                 │
│  │ • FK Solver │    │ • Safety    │    │ • PUB-SUB   │                 │
│  │ • IK Solver │    │ • Interlocks│    │ • PUSH-PULL │                 │
│  │ • 8 Solutions│   │ • ISO 10218 │    │ • Heartbeat │                 │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                 │
│         │                  │                  │                         │
│         └────────┬─────────┴─────────┬────────┘                         │
│                  │                   │                                   │
│                  ▼                   ▼                                   │
│         ┌─────────────┐      ┌─────────────┐                            │
│         │ Trajectory  │      │   grblHAL   │                            │
│         │  Generator  │─────▶│   Driver    │                            │
│         │             │      │             │                            │
│         │ • Ruckig OTG│      │ • G-code TX │                            │
│         │ • Phase Sync│      │ • Char Count│                            │
│         │ • S-Curve   │      │ • Status RX │                            │
│         │ • MOVL/MOVJ │      │ • Bypass    │                            │
│         └─────────────┘      └──────┬──────┘                            │
│                                     │                                    │
│                                     ▼                                    │
│                            ┌─────────────┐                              │
│                            │   Teensy    │                              │
│                            │    4.1      │                              │
│                            │  (grblHAL)  │                              │
│                            └─────────────┘                              │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2. Module Dependencies

```
                      ┌──────────────┐
                      │   Config     │
                      │   System     │
                      └──────┬───────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
       ┌──────────┐   ┌──────────┐   ┌──────────┐
       │  Logger  │   │   IPC    │   │  Robot   │
       │          │   │  Layer   │   │  Model   │
       └──────────┘   └────┬─────┘   └────┬─────┘
                           │              │
                           │         ┌────┴─────┐
                           │         │Kinematics│
                           │         │  IK/FK   │
                           │         └────┬─────┘
                           │              │
              ┌────────────┴──────────────┤
              │                           │
              ▼                           ▼
       ┌──────────┐                ┌──────────┐
       │  State   │                │ Ruckig   │
       │ Manager  │◄───────────────│   OTG    │
       └────┬─────┘                └────┬─────┘
            │                           │
       ┌────┴────────────┬──────────────┤
       │                 │              │
       ▼                 ▼              ▼
  ┌─────────┐     ┌──────────┐   ┌──────────┐
  │ Welding │     │   Jog    │   │ grblHAL  │
  │Sequencer│     │  Mode    │   │ Driver   │
  └────┬────┘     └──────────┘   └──────────┘
       │
       ▼
  ┌─────────┐
  │ Vision  │
  │Pipeline │
  └─────────┘
```

### 3.3. Build Order

Thứ tự build đảm bảo dependencies được resolve:

```
1.  Config System      (no deps)
2.  Logger             (Config)
3.  IPC Layer          (Config, Logger)
4.  Robot Model        (Config)
5.  Kinematics         (Robot Model, Eigen)
6.  State Manager      (Config, IPC)
7.  Trajectory/Ruckig  (Kinematics, State Manager)
8.  grblHAL Driver     (Trajectory, IPC)
9.  Jog Mode           (Kinematics, State Manager)
10. Welding Sequencer  (State Manager, I/O)
11. Vision Pipeline    (Config, Point Cloud libs)
```

---

## 4. Data Flow Architecture

### 4.1. Command Flow (User → Robot)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          COMMAND FLOW                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────┐                                                            │
│  │  User   │                                                            │
│  │ Action  │  (e.g., Press JOG+ button)                                 │
│  └────┬────┘                                                            │
│       │                                                                  │
│       ▼                                                                  │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  UI LAYER (C#)                                                   │   │
│  │                                                                   │   │
│  │  JogViewModel                                                     │   │
│  │  └── Validate input (Enable pressed?)                            │   │
│  │      └── Create JogCommand object                                 │   │
│  │          └── IPCClient.Send(JogCommand)                          │   │
│  └────────────────────────────────┬────────────────────────────────┘   │
│                                   │                                     │
│                          ZeroMQ REQ-REP                                 │
│                                   │                                     │
│  ┌────────────────────────────────▼────────────────────────────────┐   │
│  │  CORE LAYER (C++)                                                │   │
│  │                                                                   │   │
│  │  IPCServer                                                        │   │
│  │  └── Deserialize JogCommand                                       │   │
│  │      └── StateManager.RequestJog()                                │   │
│  │          └── Check FSM state (must be Ready/Jog)                  │   │
│  │              └── Safety.CheckLimits()                             │   │
│  │                  └── Kinematics.CalculateJogTarget()              │   │
│  │                      └── Trajectory.GeneratePath()                │   │
│  │                          └── grblDriver.Stream()                  │   │
│  └────────────────────────────────┬────────────────────────────────┘   │
│                                   │                                     │
│                           USB Serial                                    │
│                     (Character Counting)                                │
│                                   │                                     │
│  ┌────────────────────────────────▼────────────────────────────────┐   │
│  │  FIRMWARE (Teensy)                                               │   │
│  │                                                                   │   │
│  │  grblHAL                                                          │   │
│  │  └── Parse G-code (G1 J0:x J1:y J2:z ...)                        │   │
│  │      └── Segment Buffer                                           │   │
│  │          └── Step Generator (ISR @ 1MHz)                          │   │
│  │              └── Step/Dir Pulses → Servo Drivers                 │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.2. Status Flow (Robot → User)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          STATUS FLOW                                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  FIRMWARE (Teensy)                                               │   │
│  │                                                                   │   │
│  │  grblHAL Status Report                                            │   │
│  │  └── <Idle|MPos:0.00,0.00,0.00,0.00,0.00,0.00|FS:0,0>           │   │
│  │                                                                   │   │
│  └────────────────────────────────┬────────────────────────────────┘   │
│                                   │                                     │
│                           USB Serial                                    │
│                           (100Hz poll)                                  │
│                                   │                                     │
│  ┌────────────────────────────────▼────────────────────────────────┐   │
│  │  CORE LAYER (C++)                                                │   │
│  │                                                                   │
│  │  grblDriver.ParseStatus()                                         │   │
│  │  └── Update RobotState object                                     │   │
│  │      └── Kinematics.FK(JointAngles) → CartesianPose             │   │
│  │          └── StateManager.UpdateState()                           │   │
│  │              └── IPCServer.Publish(RobotState)                    │   │
│  └────────────────────────────────┬────────────────────────────────┘   │
│                                   │                                     │
│                         ZeroMQ PUB-SUB                                  │
│                          (Multicast)                                    │
│                                   │                                     │
│  ┌────────────────────────────────▼────────────────────────────────┐   │
│  │  UI LAYER (C#)                                                   │   │
│  │                                                                   │   │
│  │  IPCClient (Subscriber)                                           │   │
│  │  └── Receive RobotState @ 100Hz                                   │   │
│  │      └── RobotStateThrottler                                      │   │
│  │          └── Rx.NET Sample(33ms) → 30Hz                          │   │
│  │              └── ViewModel.UpdateUI()                             │   │
│  │                  └── 3D View, Coordinates, Status                │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.3. Trajectory Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      TRAJECTORY GENERATION FLOW                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Input: Target Pose (X, Y, Z, Rx, Ry, Rz)                               │
│         Motion Type: MOVJ / MOVL / MOVC                                 │
│         Speed: 50% override                                              │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  1. KINEMATICS                                                   │   │
│  │                                                                   │   │
│  │  MOVJ Path:                          MOVL Path:                   │   │
│  │  ┌─────────┐                         ┌─────────┐                 │   │
│  │  │ Target  │                         │ Target  │                 │   │
│  │  │  Pose   │                         │  Pose   │                 │   │
│  │  └────┬────┘                         └────┬────┘                 │   │
│  │       │                                   │                       │   │
│  │       ▼                                   ▼                       │   │
│  │  ┌─────────┐                         ┌─────────┐                 │   │
│  │  │   IK    │                         │ Interp  │ (Cartesian)     │   │
│  │  │ Solver  │                         │  Path   │                 │   │
│  │  └────┬────┘                         └────┬────┘                 │   │
│  │       │ (8 solutions)                     │ (N waypoints)         │   │
│  │       ▼                                   ▼                       │   │
│  │  ┌─────────┐                         ┌─────────┐                 │   │
│  │  │ Config  │                         │ IK each │                 │   │
│  │  │ Select  │                         │waypoint │                 │   │
│  │  └────┬────┘                         └────┬────┘                 │   │
│  │       │                                   │                       │   │
│  │       └───────────────┬───────────────────┘                       │   │
│  │                       │                                           │   │
│  │                       ▼                                           │   │
│  │               Joint Space Path                                    │   │
│  │               [q0, q1, ... qN]                                    │   │
│  └───────────────────────┬─────────────────────────────────────────┘   │
│                          │                                              │
│  ┌───────────────────────▼─────────────────────────────────────────┐   │
│  │  2. TRAJECTORY (Ruckig OTG)                                      │   │
│  │                                                                   │   │
│  │  Input:  [q_start] → [q_end]                                     │   │
│  │          v_max, a_max, j_max per joint                           │   │
│  │          Synchronization: Phase (MOVL) / Time (MOVJ)             │   │
│  │                                                                   │   │
│  │  ┌────────────────────────────────────────────────────────────┐  │   │
│  │  │                                                             │  │   │
│  │  │  Position ──────────────────────────────────────────────   │  │   │
│  │  │            ╱                                         ╲     │  │   │
│  │  │           ╱                                           ╲    │  │   │
│  │  │          ╱ Accel      Cruise (optional)      Decel     ╲   │  │   │
│  │  │         ╱                                               ╲  │  │   │
│  │  │  ──────╱─────────────────────────────────────────────────╲─│  │   │
│  │  │                                                             │  │   │
│  │  │  Velocity ────────────╱╲──────────────────────╱╲──────────  │  │   │
│  │  │                      ╱  ╲                    ╱  ╲          │  │   │
│  │  │                     ╱    ╲__________________╱    ╲         │  │   │
│  │  │                    ╱                              ╲        │  │   │
│  │  │                   ╱                                ╲       │  │   │
│  │  │  ────────────────╱                                  ╲──────│  │   │
│  │  │                                                             │  │   │
│  │  │  S-Curve Profile (Jerk-Limited)                            │  │   │
│  │  └────────────────────────────────────────────────────────────┘  │   │
│  │                                                                   │   │
│  │  Output: Stream of (position, velocity, acceleration) @ 10ms    │   │
│  └───────────────────────┬─────────────────────────────────────────┘   │
│                          │                                              │
│  ┌───────────────────────▼─────────────────────────────────────────┐   │
│  │  3. G-CODE STREAMING                                             │   │
│  │                                                                   │   │
│  │  Convert to G-code:                                               │   │
│  │  G1 J0:45.123 J1:-30.456 J2:60.789 J3:0.000 J4:90.123 J5:0.000  │   │
│  │     F:3600                                                        │   │
│  │                                                                   │   │
│  │  Character Counting Protocol:                                     │   │
│  │  ┌─────────────────────────────────────────────────────────────┐ │   │
│  │  │ Buffer: 4096 bytes                    Available: 3500       │ │   │
│  │  │                                                              │ │   │
│  │  │ TX Queue: [G1...] [G1...] [G1...] ─────────────▶ MCU       │ │   │
│  │  │                                                              │ │   │
│  │  │ On "ok": available += len(sent_cmd)                         │ │   │
│  │  └─────────────────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Communication Protocols

### 5.1. IPC Protocol (C++ ↔ C#)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         IPC PROTOCOL (ZeroMQ)                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  SOCKET TOPOLOGY                                                 │   │
│  │                                                                   │   │
│  │     C++ Core                              C# UI                   │   │
│  │  ┌───────────────┐                   ┌───────────────┐           │   │
│  │  │               │                   │               │           │   │
│  │  │  REP Socket   │◄───REQ-REP───────▶│  REQ Socket   │           │   │
│  │  │ tcp://*:5555  │   (Commands)      │ tcp://...:5555│           │   │
│  │  │               │                   │               │           │   │
│  │  │  PUB Socket   │────PUB-SUB───────▶│  SUB Socket   │           │   │
│  │  │ tcp://*:5556  │   (State)         │ tcp://...:5556│           │   │
│  │  │               │                   │               │           │   │
│  │  │  PULL Socket  │◄───PUSH-PULL─────▶│  PUSH Socket  │           │   │
│  │  │ tcp://*:5557  │   (Logs/Events)   │ tcp://...:5557│           │   │
│  │  │               │                   │               │           │   │
│  │  └───────────────┘                   └───────────────┘           │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  MESSAGE FORMAT (Binary)                                         │   │
│  │                                                                   │   │
│  │  ┌────────────────────────────────────────────────────────────┐  │   │
│  │  │ Header (16 bytes)                                          │  │   │
│  │  │ ┌──────────┬──────────┬──────────┬──────────┬────────────┐│  │   │
│  │  │ │  Magic   │ Version  │  Type    │ Payload  │  Checksum  ││  │   │
│  │  │ │ (4B)     │ (2B)     │ (2B)     │ Len (4B) │  (4B)      ││  │   │
│  │  │ │ 0x524F42 │ 0x0100   │ 0x0001   │ varies   │  CRC32     ││  │   │
│  │  │ └──────────┴──────────┴──────────┴──────────┴────────────┘│  │   │
│  │  │ Payload (N bytes)                                          │  │   │
│  │  │ ┌────────────────────────────────────────────────────────┐│  │   │
│  │  │ │ MessagePack / JSON / Binary struct                     ││  │   │
│  │  │ └────────────────────────────────────────────────────────┘│  │   │
│  │  └────────────────────────────────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  MESSAGE TYPES                                                   │   │
│  │                                                                   │   │
│  │  Commands (REQ-REP):          State (PUB-SUB):                   │   │
│  │  ├── 0x0001: JOG_START        ├── 0x1001: ROBOT_STATE            │   │
│  │  ├── 0x0002: JOG_STOP         ├── 0x1002: ALARM_STATE            │   │
│  │  ├── 0x0003: MOVE_TO          ├── 0x1003: WELDING_STATE          │   │
│  │  ├── 0x0004: PROGRAM_RUN      └── 0x1004: IO_STATE               │   │
│  │  ├── 0x0005: PROGRAM_STOP                                        │   │
│  │  ├── 0x0006: SET_CONFIG                                          │   │
│  │  └── 0x0007: GET_CONFIG                                          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 5.2. Serial Protocol (PC ↔ MCU)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SERIAL PROTOCOL (grblHAL)                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Connection: USB CDC @ 115200 baud                                      │
│  Flow Control: Character Counting (Token Bucket)                        │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  TX (PC → MCU): G-code Commands                                  │   │
│  │                                                                   │   │
│  │  Motion:                                                          │   │
│  │  G1 J0:45.123 J1:-30.456 J2:60.789 J3:0.0 J4:90.0 J5:0.0 F3600  │   │
│  │                                                                   │   │
│  │  Real-time Commands (immediate, no buffer):                       │   │
│  │  0x18  Feed Hold (!)                                              │   │
│  │  0x85  Jog Cancel                                                 │   │
│  │  0x84  Safety Door                                                │   │
│  │  ?     Status Query                                               │   │
│  │                                                                   │   │
│  │  Settings:                                                        │   │
│  │  $100=200.0  (Steps/unit for axis 0)                             │   │
│  │  $110=180.0  (Max rate for axis 0)                               │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  RX (MCU → PC): Responses & Status                               │   │
│  │                                                                   │   │
│  │  Acknowledgment:                                                  │   │
│  │  ok                    (Command executed)                         │   │
│  │  error:X               (Error code X)                             │   │
│  │                                                                   │   │
│  │  Status Report (@ 10Hz):                                          │   │
│  │  <Idle|MPos:0.00,0.00,0.00,0.00,0.00,0.00|FS:0,0|Pn:XYZ>         │   │
│  │                                                                   │   │
│  │  Fields:                                                          │   │
│  │  ├── State: Idle/Run/Hold/Jog/Alarm/Door/Check/Home/Sleep        │   │
│  │  ├── MPos: Machine position (6 joints, degrees)                  │   │
│  │  ├── FS: Feed rate, Spindle speed                                │   │
│  │  ├── Pn: Input pin states (limits, probes)                       │   │
│  │  └── Ov: Override values (feed, rapid, spindle)                  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  CHARACTER COUNTING FLOW CONTROL                                 │   │
│  │                                                                   │   │
│  │  RX_BUFFER_SIZE = 4096 bytes (on MCU)                            │   │
│  │                                                                   │   │
│  │  PC maintains: available_space = 4096                             │   │
│  │                                                                   │   │
│  │  On Send:                                                         │   │
│  │    if (cmd.length <= available_space):                           │   │
│  │        serial.write(cmd)                                          │   │
│  │        available_space -= cmd.length                              │   │
│  │                                                                   │   │
│  │  On Receive "ok":                                                 │   │
│  │        available_space += oldest_pending_cmd.length               │   │
│  │                                                                   │   │
│  │  Benefits:                                                        │   │
│  │  ├── Continuous streaming (no wait-for-ok latency)               │   │
│  │  ├── Buffer never overflows                                       │   │
│  │  └── Maximum throughput                                           │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 6. State Machine Architecture

### 6.1. System State Machine (ISO 10218-1)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     SYSTEM STATE MACHINE                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│                              ┌─────────┐                                │
│                              │  INIT   │                                │
│                              └────┬────┘                                │
│                                   │ Power On                            │
│                                   ▼                                     │
│                              ┌─────────┐                                │
│                         ┌───▶│  IDLE   │◀───┐                           │
│                         │    └────┬────┘    │                           │
│                         │         │         │                           │
│                    Reset│    Motor│ON       │Motor OFF                  │
│                         │         ▼         │                           │
│                         │    ┌─────────┐    │                           │
│                         │    │  READY  │────┘                           │
│                         │    └────┬────┘                                │
│                         │         │                                     │
│           ┌─────────────┼─────────┼─────────────┐                       │
│           │             │         │             │                       │
│           ▼             │         ▼             ▼                       │
│      ┌─────────┐        │    ┌─────────┐   ┌─────────┐                 │
│      │   JOG   │        │    │   RUN   │   │TEACHING │                 │
│      │  Mode   │        │    │  Mode   │   │  Mode   │                 │
│      └────┬────┘        │    └────┬────┘   └────┬────┘                 │
│           │             │         │             │                       │
│           │             │         │             │                       │
│           │      ┌──────┴─────────┴─────────────┘                       │
│           │      │                                                      │
│           │      │  E-Stop / Fault / Safety Door                       │
│           │      │                                                      │
│           │      ▼                                                      │
│           │ ┌─────────┐         ┌─────────┐                            │
│           │ │  HOLD   │◀───────▶│  ALARM  │                            │
│           │ │ (Cat 2) │ Fault   │ (Cat 0) │                            │
│           │ └────┬────┘         └────┬────┘                            │
│           │      │                   │                                  │
│           └──────┴───────────────────┘                                  │
│                         │                                               │
│                    Reset│                                               │
│                         ▼                                               │
│                   (Back to IDLE)                                        │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  STOP CATEGORIES (IEC 60204-1)                                   │   │
│  │                                                                   │   │
│  │  Category 0: Immediate power removal (E-Stop)                    │   │
│  │              → Motors de-energized instantly                      │   │
│  │              → May cause mechanical stress                        │   │
│  │                                                                   │   │
│  │  Category 1: Controlled stop, then power removal                 │   │
│  │              → Decelerate to zero velocity                        │   │
│  │              → Then remove power                                  │   │
│  │              → Safety Door trigger                                │   │
│  │                                                                   │   │
│  │  Category 2: Controlled stop, power maintained                   │   │
│  │              → Decelerate to zero velocity                        │   │
│  │              → Keep power for position holding                    │   │
│  │              → Feed Hold during operation                         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.2. State Transition Table

| Current State | Event | Next State | Action |
|--------------|-------|------------|--------|
| INIT | Config loaded | IDLE | Initialize all modules |
| IDLE | Motor ON | READY | Enable servo drivers |
| READY | Jog request | JOG | Start jog mode |
| READY | Program start | RUN | Execute program |
| READY | Teach mode | TEACHING | Enable point recording |
| JOG/RUN/TEACHING | E-Stop | ALARM | Cat 0 stop, disable all |
| JOG/RUN/TEACHING | Safety Door | HOLD | Cat 1 controlled stop |
| JOG/RUN/TEACHING | Soft Limit | HOLD | Cat 2 controlled stop |
| HOLD | Resume | (previous) | Continue operation |
| HOLD | Reset | IDLE | Clear and restart |
| ALARM | Reset + Fault cleared | IDLE | Re-initialize |

---

## 7. Safety Architecture

### 7.1. Safety Layers

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        SAFETY ARCHITECTURE                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  LAYER 4: UI SAFETY (Preventive)                                 │   │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │   │
│  │  │ Two-Handed │ │ Enable     │ │ Override   │ │ Visual     │    │   │
│  │  │ Operation  │ │ Button Req │ │ Limit (T1) │ │ Warnings   │    │   │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│  ┌─────────────────────────────────▼───────────────────────────────┐   │
│  │  LAYER 3: CORE SAFETY (Watchdog)                                 │   │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │   │
│  │  │ Heartbeat  │ │ Soft Limit │ │ Velocity   │ │ Command    │    │   │
│  │  │ Monitor    │ │ Check      │ │ Monitor    │ │ Validation │    │   │
│  │  │ (150ms)    │ │            │ │            │ │            │    │   │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│  ┌─────────────────────────────────▼───────────────────────────────┐   │
│  │  LAYER 2: FIRMWARE SAFETY (Real-time)                           │   │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │   │
│  │  │ Hard Limit │ │ Watchdog   │ │ Buffer     │ │ Step Pulse │    │   │
│  │  │ Switches   │ │ Timer      │ │ Underrun   │ │ Integrity  │    │   │
│  │  │            │ │ (ISR)      │ │ Detection  │ │ Check      │    │   │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│  ┌─────────────────────────────────▼───────────────────────────────┐   │
│  │  LAYER 1: HARDWARE SAFETY (Fail-safe)                           │   │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │   │
│  │  │ E-Stop     │ │ Safety     │ │ Dual-      │ │ Motor      │    │   │
│  │  │ Circuit    │ │ Relay      │ │ Channel    │ │ Brake      │    │   │
│  │  │ (NC)       │ │ (STO)      │ │ Inputs     │ │ (Gravity)  │    │   │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  Standards Compliance:                                                  │
│  ├── ISO 10218-1: Safety requirements for industrial robots            │
│  ├── ISO 13849-1: Safety-related parts of control systems (PLd)       │
│  └── IEC 60204-1: Safety of machinery - Electrical equipment           │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 7.2. Safety Signal Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      SAFETY SIGNAL FLOW                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Physical Inputs                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐                │
│  │ E-Stop   │  │  Safety  │  │  Limit   │  │ Deadman  │                │
│  │ Button   │  │   Door   │  │ Switches │  │ Switch   │                │
│  │   (NC)   │  │   (NC)   │  │   (NC)   │  │  (3-pos) │                │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘                │
│       │             │             │             │                       │
│       └──────┬──────┴──────┬──────┴──────┬──────┘                       │
│              │             │             │                              │
│              ▼             ▼             ▼                              │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  TEENSY 4.1 SAFETY I/O                                          │   │
│  │                                                                   │   │
│  │  ┌──────────────────────────────────────────────────────────┐   │   │
│  │  │  Dual-Channel Input Processing                           │   │   │
│  │  │                                                           │   │   │
│  │  │  Channel A ───┬───▶ Compare ◀───┬─── Channel B           │   │   │
│  │  │               │        │        │                         │   │   │
│  │  │               │    Mismatch?    │                         │   │   │
│  │  │               │        │        │                         │   │   │
│  │  │               │   Yes: ALARM    │                         │   │   │
│  │  │               │   No: Process   │                         │   │   │
│  │  └──────────────────────────────────────────────────────────┘   │   │
│  │                                                                   │   │
│  │  Safety Actions:                                                  │   │
│  │  ├── E-Stop Active → Cat 0 Stop (immediate motor disable)       │   │
│  │  ├── Door Open → Cat 1 Stop (controlled decel, then disable)    │   │
│  │  ├── Limit Hit → Cat 2 Stop (controlled stop, maintain power)   │   │
│  │  └── Deadman Released → Jog Stop (immediate jog cancel)         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│              │                                                          │
│              ▼                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  SAFETY RELAY (STO - Safe Torque Off)                           │   │
│  │                                                                   │   │
│  │  ┌──────────┐     ┌──────────┐     ┌──────────┐                 │   │
│  │  │  MCU     │────▶│  Safety  │────▶│  Servo   │                 │   │
│  │  │ Control  │     │  Relay   │     │ Drivers  │                 │   │
│  │  │ Signal   │     │  (24V)   │     │ (Enable) │                 │   │
│  │  └──────────┘     └──────────┘     └──────────┘                 │   │
│  │                                                                   │   │
│  │  E-Stop → Relay OFF → All Servos Disabled                       │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 8. Deployment Architecture

### 8.1. Single-PC Deployment

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     DEPLOYMENT ARCHITECTURE                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  INDUSTRIAL PC (Windows 10/11)                                   │   │
│  │                                                                   │   │
│  │  Specifications:                                                  │   │
│  │  ├── CPU: Intel i5/i7 or AMD Ryzen 5/7 (≥4 cores)               │   │
│  │  ├── RAM: 16GB minimum                                           │   │
│  │  ├── Storage: 256GB SSD                                          │   │
│  │  ├── GPU: Integrated (Intel UHD) or Discrete (for 3D)           │   │
│  │  ├── Display: Full HD 1920x1080 Touch Screen                     │   │
│  │  └── USB: USB 3.0 for MCU connection                             │   │
│  │                                                                   │   │
│  │  ┌────────────────────────────────────────────────────────────┐  │   │
│  │  │  SOFTWARE STACK                                             │  │   │
│  │  │                                                              │  │   │
│  │  │  C:\RobotController\                                         │  │   │
│  │  │  ├── bin\                                                    │  │   │
│  │  │  │   ├── RobotController.exe    (C# WPF UI)                 │  │   │
│  │  │  │   ├── RobotCore.dll          (C++ Core)                  │  │   │
│  │  │  │   ├── libzmq.dll             (ZeroMQ)                    │  │   │
│  │  │  │   └── *.dll                  (Dependencies)              │  │   │
│  │  │  │                                                           │  │   │
│  │  │  ├── config\                                                 │  │   │
│  │  │  │   ├── robot_config.yaml      (Robot parameters)          │  │   │
│  │  │  │   ├── app_config.yaml        (App settings)              │  │   │
│  │  │  │   └── user_config.yaml       (User preferences)          │  │   │
│  │  │  │                                                           │  │   │
│  │  │  ├── programs\                  (Robot programs)             │  │   │
│  │  │  │                                                           │  │   │
│  │  │  ├── logs\                      (Log files)                  │  │   │
│  │  │  │                                                           │  │   │
│  │  │  └── models\                    (3D robot models)            │  │   │
│  │  └────────────────────────────────────────────────────────────┘  │   │
│  │                                                                   │   │
│  │  ┌────────────────────────────────────────────────────────────┐  │   │
│  │  │  PROCESS ARCHITECTURE                                       │  │   │
│  │  │                                                              │  │   │
│  │  │  ┌─────────────────┐    ┌─────────────────┐                 │  │   │
│  │  │  │  RobotController │◄──▶│    RobotCore    │                 │  │   │
│  │  │  │     .exe         │    │   (in-process   │                 │  │   │
│  │  │  │   (UI Process)   │    │    or service)  │                 │  │   │
│  │  │  └────────┬─────────┘    └────────┬────────┘                 │  │   │
│  │  │           │                       │                          │  │   │
│  │  │           │    ZeroMQ TCP         │                          │  │   │
│  │  │           │    localhost:5555-57  │                          │  │   │
│  │  │           │                       │                          │  │   │
│  │  │           └───────────┬───────────┘                          │  │   │
│  │  │                       │                                       │  │   │
│  │  │                       ▼                                       │  │   │
│  │  │              ┌─────────────────┐                             │  │   │
│  │  │              │   USB Serial    │                             │  │   │
│  │  │              │   (COM Port)    │                             │  │   │
│  │  │              └────────┬────────┘                             │  │   │
│  │  └───────────────────────┼────────────────────────────────────┘  │   │
│  └──────────────────────────┼───────────────────────────────────────┘   │
│                             │                                           │
│                             │ USB Cable                                 │
│                             │                                           │
│  ┌──────────────────────────▼───────────────────────────────────────┐   │
│  │  CONTROL BOX                                                      │   │
│  │                                                                   │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐         │   │
│  │  │ Teensy   │  │  Power   │  │  Servo   │  │  Safety  │         │   │
│  │  │   4.1    │  │  Supply  │  │ Drivers  │  │  Relay   │         │   │
│  │  │ (grblHAL)│  │ (24V/48V)│  │  (x6)    │  │   STO    │         │   │
│  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘         │   │
│  │                                                                   │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐                       │   │
│  │  │ Digital  │  │  Analog  │  │  Weld    │                       │   │
│  │  │   I/O    │  │   I/O    │  │Interface │                       │   │
│  │  │ (24VDC)  │  │ (0-10V)  │  │          │                       │   │
│  │  └──────────┘  └──────────┘  └──────────┘                       │   │
│  └───────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 8.2. File System Layout

```
C:\RobotController\
├── bin\                           # Executables
│   ├── RobotController.exe        # Main application
│   ├── RobotCore.dll              # C++ core library
│   ├── RobotCoreService.exe       # Core as Windows Service (optional)
│   └── dependencies\              # Third-party DLLs
│
├── config\                        # Configuration
│   ├── defaults\                  # Factory defaults (read-only)
│   │   └── robot_config.yaml
│   ├── machine\                   # Machine-specific (installer sets)
│   │   └── robot_config.yaml
│   ├── app_config.yaml            # Application settings
│   └── user_config.yaml           # User preferences
│
├── programs\                      # Robot programs
│   ├── examples\                  # Sample programs
│   └── user\                      # User-created programs
│
├── models\                        # 3D Models
│   ├── default_6dof\              # Default robot model
│   │   ├── base.stl
│   │   ├── link1.stl
│   │   └── ...
│   └── custom\                    # Custom robot models
│
├── logs\                          # Log files
│   ├── app\                       # Application logs
│   ├── core\                      # Core engine logs
│   └── archive\                   # Rotated logs
│
├── firmware\                      # Firmware binaries
│   └── grblHAL_teensy41.hex
│
└── docs\                          # Documentation
    ├── user_manual.pdf
    └── maintenance_guide.pdf
```

---

## 9. Technology Stack Summary

### 9.1. Complete Stack

| Layer | Component | Technology | Version |
|-------|-----------|------------|---------|
| **UI** | Framework | WPF (.NET 8) | 8.0 |
| **UI** | MVVM | CommunityToolkit.Mvvm | 8.2+ |
| **UI** | 3D Engine | Helix Toolkit SharpDX | 2.25+ |
| **UI** | IPC Client | NetMQ | 4.0+ |
| **UI** | Logging | Serilog | 3.1+ |
| **UI** | Config | YamlDotNet | 15.1+ |
| **Core** | Language | C++ 17/20 | MSVC 2022 |
| **Core** | Build | CMake | 3.20+ |
| **Core** | Kinematics | Robotics Library | Latest |
| **Core** | Trajectory | Ruckig | 0.9+ |
| **Core** | Math | Eigen | 3.4+ |
| **Core** | IPC Server | cppzmq | 4.10+ |
| **Core** | Logging | spdlog | 1.12+ |
| **Core** | Config | yaml-cpp | 0.8+ |
| **Firmware** | Platform | Teensy 4.1 | - |
| **Firmware** | Motion | grblHAL | Latest |
| **Firmware** | Build | PlatformIO | Latest |

### 9.2. External Dependencies

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       DEPENDENCY GRAPH                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  C# UI Layer:                                                            │
│  ├── HelixToolkit.Wpf.SharpDX ──► SharpDX                               │
│  ├── NetMQ ──► libzmq                                                   │
│  ├── CommunityToolkit.Mvvm                                              │
│  ├── Serilog ──► Serilog.Sinks.*                                        │
│  └── YamlDotNet                                                         │
│                                                                          │
│  C++ Core Layer:                                                         │
│  ├── Robotics Library (RL) ──► Eigen, libxml2                           │
│  ├── Ruckig (header-only)                                               │
│  ├── Eigen (header-only)                                                │
│  ├── cppzmq ──► libzmq                                                  │
│  ├── spdlog (header-only) ──► fmt                                       │
│  └── yaml-cpp                                                           │
│                                                                          │
│  Firmware:                                                               │
│  └── grblHAL ──► Teensy Core                                            │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 10. References

### 10.1. Internal Documents

| Document | Description |
|----------|-------------|
| [00_MASTER_ROADMAP.md](./00_MASTER_ROADMAP.md) | Project roadmap and phases |
| [CORE_01_Project_Setup.md](./core/CORE_01_Project_Setup.md) | Build system configuration |
| [CORE_IPC.md](./core/CORE_IPC.md) | IPC layer design |
| [CORE_Config.md](./core/CORE_Config.md) | Configuration system |
| [CORE_StateManager.md](./core/CORE_StateManager.md) | State machine design |
| [CORE_Kinematics.md](./core/CORE_Kinematics.md) | IK/FK implementation |
| [CORE_Trajectory.md](./core/CORE_Trajectory.md) | Ruckig OTG integration |
| [CORE_grblHAL.md](./core/CORE_grblHAL.md) | Firmware interface |
| [CORE_08_HMI_Framework.md](./core/CORE_08_HMI_Framework.md) | HMI design |

### 10.2. Research Documents

| Document | Topic |
|----------|-------|
| PROJECT BLUEPRINT | Overall architecture |
| Thiết kế HMI Robot KUKA WPF | HMI design patterns |
| Thiết Kế FSM Robot Công Nghiệp | State machine design |
| Tích hợp Ruckig cho Robot Hàn | Trajectory generation |
| Tối ưu grblHAL cho Robot 6-DOF | Firmware customization |

### 10.3. Standards

| Standard | Description |
|----------|-------------|
| ISO 10218-1 | Safety requirements for industrial robots |
| ISO 13849-1 | Safety-related parts of control systems |
| IEC 60204-1 | Safety of machinery - Electrical equipment |

---

*Document version: 1.0 | Last updated: 2026-02-01*
