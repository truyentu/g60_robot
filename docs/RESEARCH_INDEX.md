# Research Papers Index

## Cách sử dụng

Khi gặp vấn đề kỹ thuật không có trong Required Reading của IMPL plan:
1. Tìm keyword trong bảng dưới
2. Đọc paper tương ứng trong `ressearch_doc_md/`
3. Nếu không tìm thấy paper phù hợp → mới search web

---

## Index by Topic

| Topic | Keywords | Paper | File |
|-------|----------|-------|------|
| **Architecture** | blueprint, commercial, system design, overview | PROJECT BLUEPRINT: BỘ ĐIỀU KHIỂN ROBOT HÀN 6-DOF | `PROJECT BLUEPRINT_ BỘ ĐIỀU KHIỂN ROBOT HÀN 6-DOF THƯƠNG MẠI.md` |
| **Robot Config** | KUKA, $machine.dat, $robcor.dat, DH parameters, catalog, multi-robot, WorkVisual | KUKA Robot Configuration Architecture | `KUKA_Robot_Configuration_Architecture.md` |
| **Platform** | open source, framework, comparison, ROS alternatives | Tìm kiếm nền tảng robot hàn mã nguồn mở | `Tìm kiếm nền tảng robot hàn mã nguồn mở.md` |
| **Kinematics** | IK, FK, DH parameters, joint limits, analytical IK | Robotics Library: Robot Tùy Chỉnh & IK Giải Tích | `Robotics Library_ Robot Tùy Chỉnh & IK Giải Tích.md` |
| **Trajectory** | Ruckig, OTG, jerk-limited, time-optimal, motion profile | Tích hợp Ruckig cho Robot Hàn 6-DOF | `Tích hợp Ruckig cho Robot Hàn 6-DOF.md` |
| **Firmware** | grblHAL, Teensy, step/dir, encoder, realtime | Tối ưu grblHAL cho Robot 6-DOF | `Tối ưu grblHAL cho Robot 6-DOF.md` |
| **State Machine** | FSM, states, transitions, safety, modes | Thiết Kế FSM Robot Công Nghiệp An Toàn | `Thiết Kế FSM Robot Công Nghiệp An Toàn.md` |
| **HMI Design** | WPF, MVVM, UI, KUKA style, controls | Thiết kế HMI Robot KUKA WPF | `Thiết kế HMI Robot KUKA WPF.md` |
| **3D Simulation** | Helix3D, visualization, 3D view, render, viewport | Thiết Kế Mô Phỏng Robot WPF Helix | `Thiết Kế Mô Phỏng Robot WPF Helix.md` |
| **Welding Control** | MIG, MAG, arc, wire feed, voltage, current | Thiết Kế Module Điều Khiển Hàn MIG_MAG | `Thiết Kế Module Điều Khiển Hàn MIG_MAG.md` |
| **Welding Programming** | weave pattern, torch, seam, welding sequence | Lập Trình Robot Hàn Chuyên Nghiệp | `Lập Trình Robot Hàn Chuyên Nghiệp.md` |
| **Vision Sensor** | laser profiler, camera, Hikrobot, MVS SDK | Robot Hàn: Cảm Biến Laser & Mã Nguồn | `Robot Hàn_ Cảm Biến Laser & Mã Nguồn.md` |
| **Scan-to-Path** | point cloud, 3D reconstruction, surface scan | Robot Hàn Scan-to-Path: Tái tạo 3D | `Robot Hàn Scan-to-Path_ Tái tạo 3D.md` |
| **3D Pathfinding** | mesh, path planning, surface following | Robot Hàn: Tái tạo 3D và Tìm đường | `Robot Hàn_ Tái tạo 3D và Tìm đường.md` |
| **Pick & Place** | gripper, pick, place, mode switching | Robot Welding sang Pick & Place | `Robot Welding sang Pick & Place.md` |
| **CAD/Tube** | OpenCASCADE, STEP, tube geometry, BREP | OpenCASCADE Tube Geometry Conversion | `OpenCASCADE Tube Geometry Conversion.md` |

---

## Index by IMPL Plan

Quick reference: Paper nào cần cho IMPL plan nào?

| IMPL Plan | Required Papers |
|-----------|-----------------|
| P1_01 ProjectSetup | PROJECT BLUEPRINT |
| P1_02 IpcLayer | PROJECT BLUEPRINT |
| P1_03 ConfigLogging | (standard libs - no paper needed) |
| P1_04 HmiVisualization | Thiết kế HMI KUKA WPF, Thiết Kế Mô Phỏng WPF Helix |
| P2_01 StateMachine | Thiết Kế FSM Robot Công Nghiệp An Toàn |
| P2_02 Kinematics | Robotics Library: IK Giải Tích |
| P2_03 Trajectory | Tích hợp Ruckig cho Robot Hàn 6-DOF |
| P2_04 FirmwareComm | Tối ưu grblHAL cho Robot 6-DOF |
| P2_05 MotionHMI | Thiết kế HMI KUKA WPF |
| P3_01 WeldingSequencer | Thiết Kế Module Điều Khiển Hàn MIG_MAG |
| P3_02 WeavingPatterns | Lập Trình Robot Hàn Chuyên Nghiệp |
| P3_03 WeldingHMI | Thiết kế HMI KUKA WPF, Thiết Kế Module Hàn |
| P4_01 SensorDrivers | Robot Hàn: Cảm Biến Laser & Mã Nguồn |
| P4_02 SeamDetection | Cảm Biến Laser, Scan-to-Path, Tái tạo 3D |
| P4_03 VisionHMI | Thiết kế HMI KUKA WPF, Cảm Biến Laser |

---

## Paper Summaries

### Architecture & Platform
- **PROJECT BLUEPRINT**: Tổng quan kiến trúc hệ thống, design decisions, công nghệ chọn dùng
- **Tìm kiếm nền tảng**: So sánh các framework robot, lý do không dùng ROS

### Robot Configuration
- **KUKA Robot Configuration Architecture**: Kiến trúc cấu hình multi-robot của KUKA, bao gồm $machine.dat, $robcor.dat, DH parameters, Robot Catalog, WorkVisual workflow. Reference cho thiết kế hệ thống Robot Catalog của project.

### Motion Control
- **Robotics Library IK**: Analytical IK cho 6-DOF, DH parameters, singularity handling
- **Tích hợp Ruckig**: Time-optimal trajectory, jerk-limited profiles, online replanning
- **Tối ưu grblHAL**: Step/dir generation, encoder feedback, realtime control loop

### Safety & Control
- **FSM An Toàn**: State machine design, emergency stop, mode transitions, interlocks

### HMI & Visualization
- **HMI KUKA WPF**: MVVM pattern, control layouts, industrial UI design
- **Mô Phỏng Helix**: 3D viewport setup, robot model loading, FK visualization

### Welding
- **Module Hàn MIG_MAG**: Arc control, wire feed, gas flow, timing sequences
- **Lập Trình Hàn**: Weave patterns, torch angles, seam following strategies

### Vision
- **Cảm Biến Laser**: Hikrobot SDK, profile acquisition, hand-eye calibration
- **Scan-to-Path**: Point cloud processing, surface reconstruction
- **Tái tạo 3D và Tìm đường**: Mesh generation, path planning on surfaces

### Utilities
- **Pick & Place**: Mode switching, gripper control, pick/place sequences
- **OpenCASCADE Tube**: CAD geometry processing, tube cutting paths
