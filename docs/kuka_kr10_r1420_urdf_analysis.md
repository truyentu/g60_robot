# KUKA KR10 R1420 URDF/XACRO Transformation Data

**Source:** [ROS-Industrial kuka_experimental repository](https://github.com/ros-industrial/kuka_experimental/blob/melodic-devel/kuka_kr10_support/urdf/kr10r1420_macro.xacro)

**Date Extracted:** 2026-02-05

---

## 1. Joint Transformation Summary

| Joint Name | Type | Parent Link | Child Link | Translation (xyz) [m] | Rotation (rpy) [rad] | Axis |
|------------|------|-------------|------------|----------------------|---------------------|------|
| joint_a1 | revolute | base_link | link_1 | `0 0 0.450` | `0 0 0` | `0 0 -1` |
| joint_a2 | revolute | link_1 | link_2 | `0.150 0 0` | `0 0 0` | `0 1 0` |
| joint_a3 | revolute | link_2 | link_3 | `0.610 0 0` | `0 0 0` | `0 1 0` |
| joint_a4 | revolute | link_3 | link_4 | `0 0 0.02` | `0 0 0` | `-1 0 0` |
| joint_a5 | revolute | link_4 | link_5 | `0.660 0 0` | `0 0 0` | `0 1 0` |
| joint_a6 | revolute | link_5 | link_6 | `0.080 0 0` | `0 0 0` | `-1 0 0` |
| joint_a6-flange | fixed | link_6 | flange | `0 0 0` | `0 0 0` | N/A |

---

## 2. Joint Limits

| Joint | Lower Limit [deg] | Upper Limit [deg] | Max Velocity [deg/s] |
|-------|-------------------|-------------------|---------------------|
| joint_a1 | -170 | +170 | 220 |
| joint_a2 | -185 | +65 | 210 |
| joint_a3 | -137 | +163 | 270 |
| joint_a4 | -185 | +185 | 381 |
| joint_a5 | -120 | +120 | 311 |
| joint_a6 | -350 | +350 | 492 |

---

## 3. Visual Mesh Origins (All Links)

All visual meshes have the same origin relative to their parent link:

| Link Name | Visual Origin (xyz) [m] | Visual Origin (rpy) [rad] | Mesh File Path |
|-----------|------------------------|--------------------------|----------------|
| base_link | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/base_link.stl` |
| link_1 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_1.stl` |
| link_2 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_2.stl` |
| link_3 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_3.stl` |
| link_4 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_4.stl` |
| link_5 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_5.stl` |
| link_6 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/visual/link_6.stl` |

**Note:** All visual meshes are positioned at the link's origin with no rotation. The actual mesh geometry contains the correct offsets.

---

## 4. Collision Mesh Origins (All Links)

Identical to visual mesh origins:

| Link Name | Collision Origin (xyz) [m] | Collision Origin (rpy) [rad] | Mesh File Path |
|-----------|---------------------------|------------------------------|----------------|
| base_link | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/base_link.stl` |
| link_1 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_1.stl` |
| link_2 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_2.stl` |
| link_3 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_3.stl` |
| link_4 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_4.stl` |
| link_5 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_5.stl` |
| link_6 | `0 0 0` | `0 0 0` | `meshes/kr10r1420/collision/link_6.stl` |

---

## 5. Transformation Tree (Kinematic Chain)

```
base_link
  └─ [joint_a1: xyz=(0, 0, 0.450)] → link_1
       └─ [joint_a2: xyz=(0.150, 0, 0)] → link_2
            └─ [joint_a3: xyz=(0.610, 0, 0)] → link_3
                 └─ [joint_a4: xyz=(0, 0, 0.02)] → link_4
                      └─ [joint_a5: xyz=(0.660, 0, 0)] → link_5
                           └─ [joint_a6: xyz=(0.080, 0, 0)] → link_6
                                └─ [fixed] → flange
                                     └─ [fixed: rpy=(0, π/2, 0)] → tool0
```

**Parallel Branch:**
```
base_link
  └─ [fixed] → base (KUKA $ROBROOT coordinate system)
```

---

## 6. Special Transforms

### 6.1 Flange to Tool0 Transform

This frame corresponds to the `$TOOL` coordinate system in KUKA KRC controllers:

| Joint Name | Type | Parent Link | Child Link | Translation (xyz) [m] | Rotation (rpy) [rad] |
|------------|------|-------------|------------|----------------------|---------------------|
| flange-tool0 | fixed | flange | tool0 | `0 0 0` | `0 1.5708 0` (0° 90° 0°) |

**Note:** The 90° rotation around Y-axis aligns the ROS convention with KUKA's tool frame convention.

### 6.2 Base Link to Base Transform

ROS `base_link` to KUKA `$ROBROOT` coordinate system:

| Joint Name | Type | Parent Link | Child Link | Translation (xyz) [m] | Rotation (rpy) [rad] |
|------------|------|-------------|------------|----------------------|---------------------|
| base_link-base | fixed | base_link | base | `0 0 0` | `0 0 0` |

---

## 7. Key Dimensions (Meters)

| Dimension | Value | Description |
|-----------|-------|-------------|
| Base Height | 0.450 | Z-offset from base_link to link_1 (joint_a1) |
| Link 1 Length | 0.150 | X-offset from link_1 to link_2 (joint_a2) |
| Link 2 Length | 0.610 | X-offset from link_2 to link_3 (joint_a3) |
| Link 3 Offset | 0.02 | Z-offset from link_3 to link_4 (joint_a4) |
| Link 4 Length | 0.660 | X-offset from link_4 to link_5 (joint_a5) |
| Link 5 Length | 0.080 | X-offset from link_5 to link_6 (joint_a6) |
| **Total Reach** | ~1.420 | Approximate maximum horizontal reach |

---

## 8. Rotation Axes Summary

| Joint | Rotation Axis | Description |
|-------|---------------|-------------|
| joint_a1 | `0 0 -1` | Negative Z-axis (base rotation) |
| joint_a2 | `0 1 0` | Positive Y-axis (shoulder) |
| joint_a3 | `0 1 0` | Positive Y-axis (elbow) |
| joint_a4 | `-1 0 0` | Negative X-axis (wrist roll) |
| joint_a5 | `0 1 0` | Positive Y-axis (wrist pitch) |
| joint_a6 | `-1 0 0` | Negative X-axis (wrist yaw/flange) |

**Note:** Axes with negative signs (joints a1, a4, a6) indicate inverse rotation direction compared to standard right-hand rule.

---

## 9. Important Notes

### 9.1 Coordinate System Conventions
- **Units:** All dimensions in URDF are in **meters**
- **Rotation Convention:** Roll-Pitch-Yaw (RPY) in **radians**
- All joint origins have zero rotation (`rpy="0 0 0"`) except the `flange-tool0` transform

### 9.2 Mesh Alignment
- All visual and collision meshes are placed at their link's origin (`xyz="0 0 0"`, `rpy="0 0 0"`)
- The actual geometry offsets are baked into the STL mesh files themselves
- This simplifies the URDF structure but means mesh files must be correctly pre-aligned

### 9.3 KUKA-Specific Frames
- **`base`**: Corresponds to KUKA's `$ROBROOT` coordinate system
- **`tool0`**: Corresponds to KUKA's `$TOOL` coordinate system with a 90° Y-rotation to match ROS conventions
- **`flange`**: Standard mounting interface following REP-199

### 9.4 Joint Naming Convention
- KUKA uses `A1-A6` notation (Axis 1-6)
- URDF uses lowercase with prefix: `joint_a1` through `joint_a6`

---

## 10. DH Parameters (Derived)

Based on the joint transforms, the approximate **Modified DH parameters** are:

| Link | α (alpha) | a | d | θ (theta) |
|------|-----------|---|---|-----------|
| 1 | 0° | 0 | 0.450 | θ₁ (variable) |
| 2 | -90° | 0.150 | 0 | θ₂ (variable) |
| 3 | 0° | 0.610 | 0 | θ₃ (variable) |
| 4 | -90° | 0 | 0.02 | θ₄ (variable) |
| 5 | 90° | 0.660 | 0 | θ₅ (variable) |
| 6 | -90° | 0.080 | 0 | θ₆ (variable) |

**Note:** These are approximate conversions from the URDF transform data. For exact DH parameters, refer to KUKA's official technical documentation.

---

## 11. File Structure

### Main Files
- **Main XACRO:** `kr10r1420.xacro` (includes the macro)
- **Macro Definition:** `kr10r1420_macro.xacro` (contains all geometry and joints)

### Dependencies
- `kuka_resources/urdf/common_materials.xacro` (material definitions)
- Mesh files in `kuka_kr10_support/meshes/kr10r1420/`

---

## 12. Source Code Reference

**GitHub Repository:** [ros-industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental)

**Direct Links:**
- Main XACRO: [kr10r1420.xacro](https://github.com/ros-industrial/kuka_experimental/blob/melodic-devel/kuka_kr10_support/urdf/kr10r1420.xacro)
- Macro Definition: [kr10r1420_macro.xacro](https://github.com/ros-industrial/kuka_experimental/blob/melodic-devel/kuka_kr10_support/urdf/kr10r1420_macro.xacro)

**Raw File URLs:**
- Main: https://raw.githubusercontent.com/ros-industrial/kuka_experimental/melodic-devel/kuka_kr10_support/urdf/kr10r1420.xacro
- Macro: https://raw.githubusercontent.com/ros-industrial/kuka_experimental/melodic-devel/kuka_kr10_support/urdf/kr10r1420_macro.xacro

---

## Changelog
- **2026-02-05:** Initial extraction and analysis
