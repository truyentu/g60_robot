# Hướng Dẫn Thêm Robot Mới Từ ROS-Industrial

## Tổng Quan

Hướng dẫn này mô tả cách thêm robot mới vào catalog từ các package ROS-Industrial trên GitHub.

## Nguồn Robot Packages

### KUKA Robots
- https://github.com/ros-industrial/kuka_experimental/tree/melodic-devel
  - `kuka_kr3_support` - KR 3 R540
  - `kuka_kr5_support` - KR 5 arc, KR 5 sixx R650, R850
  - `kuka_kr6_support` - KR 6 R700, R900
  - `kuka_kr10_support` - KR 10 R900, R1100, R1420
  - `kuka_kr16_support` - KR 16-2
  - `kuka_kr120_support` - KR 120 R2500 pro
  - `kuka_kr150_support` - KR 150-2
  - `kuka_kr210_support` - KR 210 R3100

### ABB Robots
- https://github.com/ros-industrial/abb_experimental
  - IRB series

### Fanuc Robots
- https://github.com/ros-industrial/fanuc_experimental
  - Various Fanuc models

### Universal Robots
- https://github.com/ros-industrial/universal_robot
  - UR3, UR5, UR10, UR16

---

## Bước 1: Tải Files Cần Thiết

### 1.1 Xác định robot cần thêm

Ví dụ: Thêm **KUKA KR 6 R900** từ `kuka_kr6_support`

### 1.2 Tải các files sau từ GitHub

```
kuka_kr6_support/
├── urdf/
│   └── kr6r900sixx_macro.xacro    ← URDF definitions (joint origins, axes)
├── meshes/
│   └── kr6r900sixx/
│       ├── visual/
│       │   ├── base_link.stl      ← Visual meshes
│       │   ├── link_1.stl
│       │   ├── link_2.stl
│       │   ├── link_3.stl
│       │   ├── link_4.stl
│       │   ├── link_5.stl
│       │   └── link_6.stl
│       └── collision/
│           └── ...                 ← Collision meshes (optional)
└── config/
    └── joint_limits.yaml          ← Joint limits (optional, có trong URDF)
```

### 1.3 Download Commands

```bash
# Clone repo (hoặc download ZIP)
git clone https://github.com/ros-industrial/kuka_experimental.git

# Hoặc download trực tiếp từ GitHub:
# - Go to: https://github.com/ros-industrial/kuka_experimental/tree/melodic-devel/kuka_kr6_support
# - Download meshes folder
# - Download urdf/*.xacro file
```

---

## Bước 2: Tạo Robot Package Directory

### 2.1 Tạo thư mục mới

```
src/config/robots/
└── kr6r900/                        ← Folder ID (sẽ dùng làm package ID)
    ├── robot.yaml                  ← Config file (tạo mới)
    └── meshes/
        ├── visual/
        │   ├── base.stl
        │   ├── link1.stl
        │   ├── link2.stl
        │   ├── link3.stl
        │   ├── link4.stl
        │   ├── link5.stl
        │   └── link6.stl
        └── collision/              ← Optional
            └── ...
```

### 2.2 Rename STL files (nếu cần)

ROS-Industrial thường dùng naming: `base_link.stl`, `link_1.stl`, `link_2.stl`...

Đổi thành: `base.stl`, `link1.stl`, `link2.stl`... (hoặc giữ nguyên và update robot.yaml)

---

## Bước 3: Đọc URDF/Xacro Để Lấy Thông Số

### 3.1 Mở file `.xacro`

Ví dụ từ `kr6r900sixx_macro.xacro`:

```xml
<joint name="${prefix}joint_a1" type="revolute">
  <origin xyz="0 0 0.400" rpy="0 0 0"/>
  <parent link="${prefix}base_link"/>
  <child link="${prefix}link_1"/>
  <axis xyz="0 0 -1"/>
  <limit effort="0" lower="${radians(-170)}" upper="${radians(170)}" velocity="${radians(220)}"/>
</joint>
```

### 3.2 Trích xuất thông tin cho mỗi joint

| Field | URDF Location | Ví dụ |
|-------|---------------|-------|
| `origin.xyz` | `<origin xyz="...">` | `[0, 0, 0.400]` (meters!) |
| `origin.rpy` | `<origin rpy="...">` | `[0, 0, 0]` (radians) |
| `axis` | `<axis xyz="...">` | `[0, 0, -1]` |
| `limit_min` | `<limit lower="...">` | `-170` (đổi sang degrees) |
| `limit_max` | `<limit upper="...">` | `170` (đổi sang degrees) |
| `vel_max` | `<limit velocity="...">` | `220` (đổi sang deg/s) |

### 3.3 Chuyển đổi đơn vị

**QUAN TRỌNG**: URDF dùng meters và radians, robot.yaml dùng mm và degrees!

```
# Meters → Millimeters
xyz: [0, 0, 0.400] → xyz: [0, 0, 400]

# Radians → Degrees (cho limits)
${radians(-170)} = -170 degrees (đã là degrees trong macro)

# Radians/s → Degrees/s (cho velocity)
velocity="${radians(220)}" = 220 deg/s
```

---

## Bước 4: Tìm DH Parameters

### 4.1 Nguồn DH Parameters

DH parameters thường KHÔNG có trong URDF. Tìm từ:

1. **Manufacturer datasheet** - KUKA, ABB specs
2. **Academic papers** - Search "KUKA KR6 DH parameters"
3. **ROS wiki** - Một số robot có DH params documented
4. **Existing implementations** - Các robotics libraries khác

### 4.2 Ví dụ DH Parameters cho KUKA KR6 R900

| Joint | a (mm) | α (deg) | d (mm) | θ offset (deg) |
|-------|--------|---------|--------|----------------|
| A1 | 25 | -90 | 400 | 0 |
| A2 | 455 | 0 | 0 | -90 |
| A3 | 35 | -90 | 0 | 0 |
| A4 | 0 | 90 | 420 | 0 |
| A5 | 0 | -90 | 0 | 0 |
| A6 | 0 | 0 | 80 | 0 |

---

## Bước 5: Tạo robot.yaml

### 5.1 Template

```yaml
# Robot Name - Source Info
# Hybrid approach: DH for kinematics + URDF origins for visualization

name: "KUKA KR 6 R900"
manufacturer: "KUKA"
type: "6-axis-industrial"
payload_kg: 6
reach_mm: 900

kinematics:
  convention: "modified_dh"
  joints:
    - name: "A1"
      type: "revolute"
      # DH parameters (for FK/IK calculations)
      dh:
        a: 25
        alpha: -90
        d: 400
        theta_offset: 0
      # URDF origin (for visualization) - from ROS-Industrial
      origin:
        xyz: [0, 0, 400]        # Convert meters to mm!
        rpy: [0, 0, 0]
      axis: [0, 0, -1]
      limits:
        min: -170
        max: 170
        vel_max: 220
        accel_max: 720
      mesh:
        visual: "meshes/visual/link1.stl"

    # ... repeat for A2-A6

home_position: [0, -90, 90, 0, 0, 0]

base:
  mesh: "meshes/visual/base.stl"
  origin:
    x: 0
    y: 0
    z: 0
    rx: 0
    ry: 0
    rz: 0

flange:
  offset:
    x: 0
    y: 0
    z: 0
```

---

## Bước 6: Copy Package Đến Output Directory

Robot packages cần có ở cả 2 nơi:

### 6.1 Source location (cho development)
```
src/config/robots/kr6r900/
```

### 6.2 Output location (cho runtime)
```
src/ui/RobotController.UI/bin/Debug/net8.0-windows/config/robots/kr6r900/
```

### 6.3 Tự động copy trong Visual Studio

Thêm vào `.csproj`:
```xml
<ItemGroup>
  <None Update="..\..\..\..\config\robots\**\*">
    <CopyToOutputDirectory>PreserveNewest</CopyToOutputDirectory>
    <Link>config\robots\%(RecursiveDir)%(Filename)%(Extension)</Link>
  </None>
</ItemGroup>
```

---

## Bước 7: Test Robot Package

### 7.1 Rebuild Core và UI

```bash
# Core
cd src/core/build
cmake --build . --config Release

# UI
cd src/ui
dotnet build
```

### 7.2 Chạy và test

1. Start Core
2. Start UI
3. Go to Robot Catalog
4. Select new robot
5. Load package
6. Verify visualization

### 7.3 Kiểm tra logs

```
# Core logs - verify URDF parsing
[info] Joint A1 URDF origin_xyz: [0, 0, 400]

# UI logs - verify URDF data received
Joint A1 has URDF origin_xyz: [0, 0, 400]
```

---

## Troubleshooting

### Meshes hiển thị sai vị trí

1. **Check units**: URDF dùng meters, yaml phải convert sang mm
2. **Check URDF origin**: Phải copy đúng từ `<origin xyz="...">` trong xacro
3. **Check axis**: Copy đúng từ `<axis xyz="...">`

### Meshes bị overlap/chồng lên nhau

- URDF origins bị thiếu hoặc sai
- Kiểm tra lại file xacro

### Không thấy robot trong catalog

- Check Core logs: `Found built-in package: ...`
- Verify robot.yaml syntax (YAML valid)
- Verify meshes folder exists

### STL không load được

- Check đường dẫn trong robot.yaml match với folder structure
- Verify STL files không bị corrupt

---

## Checklist Thêm Robot Mới

- [ ] Download meshes từ ROS-Industrial
- [ ] Download URDF/xacro file
- [ ] Tạo folder trong `src/config/robots/[robot-id]/`
- [ ] Copy meshes vào `meshes/visual/`
- [ ] Tạo `robot.yaml` với:
  - [ ] Metadata (name, manufacturer, payload, reach)
  - [ ] DH parameters cho mỗi joint
  - [ ] URDF origins cho mỗi joint (convert m → mm!)
  - [ ] Axis cho mỗi joint
  - [ ] Limits cho mỗi joint
  - [ ] Mesh paths
  - [ ] Home position
  - [ ] Base mesh
- [ ] Copy package đến output directory
- [ ] Rebuild Core và UI
- [ ] Test visualization
- [ ] Verify joint movements

---

## Ví Dụ: Thêm KUKA KR 16-2

### Download
```
kuka_kr16_support/meshes/kr16_2/visual/*.stl
kuka_kr16_support/urdf/kr16_2_macro.xacro
```

### Folder structure
```
src/config/robots/kr16_2/
├── robot.yaml
└── meshes/visual/
    ├── base.stl
    ├── link1.stl
    ├── link2.stl
    ├── link3.stl
    ├── link4.stl
    ├── link5.stl
    └── link6.stl
```

### Extract from xacro
```xml
<joint name="${prefix}joint_a1" type="revolute">
  <origin xyz="0 0 0.675" rpy="0 0 0"/>
  ...
</joint>
```

### robot.yaml
```yaml
name: "KUKA KR 16-2"
manufacturer: "KUKA"
payload_kg: 16
reach_mm: 1610

kinematics:
  joints:
    - name: "A1"
      origin:
        xyz: [0, 0, 675]  # 0.675m → 675mm
        rpy: [0, 0, 0]
      # ... etc
```
