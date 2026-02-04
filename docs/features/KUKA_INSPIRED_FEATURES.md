# KUKA-Inspired Features for Robot Controller

## Overview

Các tính năng được lấy cảm hứng từ KUKA robot commissioning workflow, áp dụng cho project Robot Controller.

---

## Feature 1: Robot Mastering / Homing System

### Mô tả
Hệ thống Mastering/Homing cho phép xác định vị trí "0" của từng trục robot. Đây là bước bắt buộc khi:
- Robot mới được lắp đặt
- Sau khi thay motor/encoder
- Sau khi mất nguồn đột ngột (nếu dùng incremental encoder)

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Homing Status Panel | Hiển thị trạng thái homing của từng joint |
| Homing Control | Nút Start/Stop homing cho từng joint hoặc tất cả |
| Homing Method Selector | Chọn phương pháp: Limit switch, Index pulse, Manual |
| Progress Indicator | Thanh tiến trình homing |
| Position Display | Hiển thị encoder position real-time |

### Workflow
1. User chọn method homing (Limit Switch / Index Pulse / Manual)
2. User chọn joints cần home (individual hoặc All)
3. Nhấn "Start Homing"
4. Robot di chuyển chậm về hướng home
5. Khi gặp limit switch hoặc index pulse → set position = 0
6. Hiển thị "Homed" status cho joint đó
7. Lặp lại cho các joints còn lại

### IPC Messages
- `START_HOMING` - Bắt đầu homing
- `STOP_HOMING` - Dừng homing
- `HOMING_STATUS` - Trạng thái homing (event)
- `GET_HOMING_STATE` - Query trạng thái

---

## Feature 2: Tool Calibration (TCP Management)

### Mô tả
Quản lý Tool Center Point (TCP) - điểm tác động cuối của tool gắn trên robot.

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Tool List | Danh sách các tool đã định nghĩa |
| Active Tool Indicator | Tool đang được sử dụng |
| TCP Editor | Form nhập/sửa TCP offset (X, Y, Z, Rx, Ry, Rz) |
| Calibration Wizard | Wizard hướng dẫn calibrate TCP |
| Tool Preview | 3D preview của tool trên robot |

### Calibration Methods
1. **Direct Input**: Nhập trực tiếp offset từ CAD/datasheet
2. **4-Point Method**: Chạm 4 điểm vào 1 reference tip → tính TCP
3. **6-Point Method**: Chạm 6 điểm (thêm orientation)

### Data Structure
```yaml
tool:
  id: "tool_001"
  name: "Welding Torch MIG"
  tcp:
    x: 0.0      # mm
    y: 0.0      # mm
    z: 150.0    # mm
    rx: 0.0     # degrees
    ry: 0.0     # degrees
    rz: 0.0     # degrees
  mass: 2.5     # kg
  cog:          # Center of Gravity
    x: 0.0
    y: 0.0
    z: 75.0
```

### IPC Messages
- `GET_TOOL_LIST` - Lấy danh sách tools
- `SELECT_TOOL` - Chọn active tool
- `CREATE_TOOL` - Tạo tool mới
- `UPDATE_TOOL` - Cập nhật tool
- `DELETE_TOOL` - Xóa tool
- `START_TCP_CALIBRATION` - Bắt đầu calibration wizard
- `RECORD_CALIBRATION_POINT` - Ghi điểm calibration

---

## Feature 3: Base/Workpiece Coordinate System

### Mô tả
Định nghĩa hệ tọa độ cho workpiece, cho phép lập trình theo tọa độ tương đối với workpiece thay vì world frame.

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Base List | Danh sách các base đã định nghĩa |
| Active Base Indicator | Base đang được sử dụng |
| Base Editor | Form nhập/sửa base frame |
| Calibration Wizard | Wizard 3-point calibration |
| Coordinate Display | Hiển thị tọa độ trong base frame hiện tại |

### Calibration Methods
1. **3-Point**: Origin, +X direction, XY plane point
2. **Direct Input**: Nhập trực tiếp từ CAD
3. **External Measurement**: Import từ laser tracker/CMM

### Data Structure
```yaml
base:
  id: "base_001"
  name: "Welding Table 1"
  frame:
    x: 500.0     # mm from world origin
    y: 0.0
    z: 800.0
    rx: 0.0      # degrees
    ry: 0.0
    rz: 45.0
  description: "Main welding workstation"
```

---

## Feature 4: Safety Configuration UI

### Mô tả
Giao diện cấu hình các thông số an toàn của robot.

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Joint Limits Editor | Soft limits cho từng joint (min/max) |
| Velocity Limits | Giới hạn tốc độ theo mode |
| Workspace Monitor | Vùng làm việc cho phép (3D visualization) |
| E-Stop Config | Cấu hình E-Stop inputs |
| Safety Zones | Định nghĩa vùng cấm/vùng giảm tốc |

### Data Structure
```yaml
safety:
  joint_limits:
    - joint: 1
      min: -170.0  # degrees
      max: 170.0
      velocity_max: 120.0  # deg/s
    # ... joints 2-6

  cartesian_limits:
    workspace:
      type: "cylinder"  # box, cylinder, sphere
      radius: 1200.0    # mm
      z_min: -100.0
      z_max: 1500.0

  speed_limits:
    manual_mode: 250.0     # mm/s
    auto_mode: 2000.0      # mm/s
    reduced_mode: 100.0    # mm/s

  safety_zones:
    - name: "Operator Zone"
      type: "box"
      bounds: {...}
      action: "reduce_speed"
```

---

## Feature 5: Operation Mode Management

### Mô tả
Quản lý các chế độ vận hành của robot, tương tự T1/T2/AUTO của KUKA.

### Operation Modes

| Mode | Tốc độ | Mục đích | Safety |
|------|--------|----------|--------|
| **MANUAL** | Max 250mm/s | Teaching, jogging | Deadman switch required |
| **TEST** | Full speed | Testing với người quan sát | Reduced workspace |
| **AUTO** | Full speed | Production | Full safety interlocks |
| **REMOTE** | Full speed | PLC/External control | External safety system |

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Mode Selector | Dropdown hoặc buttons chọn mode |
| Mode Status | Hiển thị mode hiện tại (màu sắc khác nhau) |
| Mode Requirements | Hiển thị điều kiện cần có để vào mode |
| Mode Transition Log | Lịch sử chuyển mode |

### IPC Messages
- `GET_OPERATION_MODE` - Query mode hiện tại
- `SET_OPERATION_MODE` - Yêu cầu chuyển mode
- `OPERATION_MODE_CHANGED` - Event khi mode thay đổi

---

## Feature 6: Program Management

### Mô tả
Quản lý các chương trình robot (teaching, waypoints, motions).

### UI Requirements

| Component | Mô tả |
|-----------|-------|
| Program List | Danh sách chương trình |
| Program Editor | Editor cho program (waypoints, commands) |
| Waypoint List | Danh sách các điểm trong program |
| Teach Button | Nút ghi vị trí hiện tại vào program |
| Run Controls | Play, Pause, Stop, Step |
| Program Status | Đang chạy dòng nào, % hoàn thành |

### Program Structure
```yaml
program:
  id: "prog_001"
  name: "Weld Seam 1"
  description: "Welding program for part A"
  waypoints:
    - id: 1
      name: "Approach"
      type: "PTP"  # PTP, LIN, CIRC
      position:
        joints: [0, -45, 90, 0, 45, 0]
      velocity: 50  # % of max
      blend: 10     # mm
    - id: 2
      name: "Weld Start"
      type: "LIN"
      position:
        cartesian: {x: 500, y: 0, z: 300, rx: 0, ry: 180, rz: 0}
      velocity: 20
      tool: "tool_001"
      base: "base_001"
```

---

## Implementation Priority

| Priority | Feature | Complexity | Dependencies |
|----------|---------|------------|--------------|
| P1 | Homing System | Medium | Motion controller |
| P2 | Tool Calibration | Medium | Kinematics |
| P3 | Base Calibration | Medium | Kinematics |
| P4 | Safety Configuration | High | Core safety module |
| P5 | Operation Modes | Low | State machine |
| P6 | Program Management | High | All above |

---

## Technical Notes

### Integration Points
- **Core C++**: Motion controller, safety module, kinematics
- **IPC**: ZeroMQ messages between Core và UI
- **UI C#**: WPF views, ViewModels
- **Config**: YAML files for persistence

### Shared Components
- Coordinate display widget (reusable)
- Position editor widget
- 3D preview integration
- Status indicator styles
