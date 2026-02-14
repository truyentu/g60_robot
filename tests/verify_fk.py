#!/usr/bin/env python3
"""
MA2010 Forward Kinematics Verification Script

Computes FK using URDF joint data with pure NumPy (manual math).
Outputs expected TCP positions and orientations for C++ GTest.

URDF FK formula per joint:
  T_joint = Translation(xyz) * RPY(rpy) * Rotation(axis, q)

For MA2010, all RPY = [0,0,0], so:
  T_joint = Translation(xyz) * Rotation(axis, q)

Units: meters in URDF, converted to mm in output for C++ tests.

Usage:
  pip install numpy
  python verify_fk.py
  python verify_fk.py --roboticstoolbox   # cross-check with RTB (optional)
"""

import numpy as np
import json
import sys
from math import pi, sin, cos

# ============================================================================
# MA2010 URDF Joint Data (from ROS-Industrial ma2010_macro.xacro)
# Units: meters, radians
# ============================================================================

MA2010_JOINTS = [
    {  # J1 (S-axis) - base rotation
        "name": "joint_1_s",
        "xyz": [0, 0, 0.505],
        "rpy": [0, 0, 0],
        "axis": [0, 0, 1],
        "limits": [-3.1416, 3.1416],
    },
    {  # J2 (L-axis) - shoulder
        "name": "joint_2_l",
        "xyz": [0.150, 0, 0],
        "rpy": [0, 0, 0],
        "axis": [0, 1, 0],
        "limits": [-1.8326, 2.7052],
    },
    {  # J3 (U-axis) - elbow
        "name": "joint_3_u",
        "xyz": [0, 0, 0.760],
        "rpy": [0, 0, 0],
        "axis": [0, -1, 0],
        "limits": [-1.5009, 2.7925],
    },
    {  # J4 (R-axis) - wrist roll
        "name": "joint_4_r",
        "xyz": [0, 0, 0.200],
        "rpy": [0, 0, 0],
        "axis": [-1, 0, 0],
        "limits": [-2.6180, 2.6180],
    },
    {  # J5 (B-axis) - wrist bend
        "name": "joint_5_b",
        "xyz": [1.082, 0, 0],
        "rpy": [0, 0, 0],
        "axis": [0, -1, 0],
        "limits": [-2.3562, 1.5708],
    },
    {  # J6 (T-axis) - tool rotation
        "name": "joint_6_t",
        "xyz": [0, 0, 0],
        "rpy": [0, 0, 0],
        "axis": [-1, 0, 0],
        "limits": [-3.6652, 3.6652],
    },
]

# Tool0 offset (fixed joint from link_6_t to tool0)
TOOL0_OFFSET = [0.100, 0, 0]  # meters


# ============================================================================
# Math Utilities
# ============================================================================

def rot_x(angle):
    """Rotation matrix about X axis"""
    c, s = cos(angle), sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

def rot_y(angle):
    """Rotation matrix about Y axis"""
    c, s = cos(angle), sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(angle):
    """Rotation matrix about Z axis"""
    c, s = cos(angle), sin(angle)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def rpy_to_rotation(rpy):
    """URDF RPY to rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    return rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])

def axis_angle_rotation(axis, angle):
    """Rodrigues' rotation formula: rotation about arbitrary axis"""
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-10:
        return np.eye(3)
    axis = axis / norm

    c, s = cos(angle), sin(angle)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return np.eye(3) + s * K + (1 - c) * (K @ K)

def make_transform(R, p):
    """Create 4x4 homogeneous transform from 3x3 rotation and 3x1 position"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def rotation_to_rpy(R):
    """Extract RPY from rotation matrix (ZYX convention, same as URDF)"""
    # Check gimbal lock
    if abs(R[2, 0]) >= 1.0 - 1e-10:
        yaw = 0.0
        if R[2, 0] < 0:
            pitch = pi / 2.0
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -pi / 2.0
            roll = np.arctan2(-R[0, 1], -R[0, 2])
    else:
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])


# ============================================================================
# URDF FK Computation
# ============================================================================

def compute_joint_transform(joint, angle):
    """
    Compute single joint transform: T = Translation(xyz) * RPY(rpy) * Rotation(axis, q)
    """
    xyz = np.array(joint["xyz"])
    rpy = np.array(joint["rpy"])
    axis = joint["axis"]

    # 1. Translation
    T = make_transform(np.eye(3), xyz)

    # 2. RPY rotation (for MA2010, all zero)
    if np.linalg.norm(rpy) > 1e-10:
        R_rpy = rpy_to_rotation(rpy)
        T_rpy = make_transform(R_rpy, np.zeros(3))
        T = T @ T_rpy

    # 3. Joint rotation about axis
    if abs(angle) > 1e-15:
        R_joint = axis_angle_rotation(axis, angle)
        T_joint = make_transform(R_joint, np.zeros(3))
        T = T @ T_joint

    return T

def compute_fk(joint_angles, joints=MA2010_JOINTS, tool_offset=TOOL0_OFFSET):
    """
    Compute full FK chain: T = T1 * T2 * T3 * T4 * T5 * T6 * T_tool

    Args:
        joint_angles: list of 6 joint angles in radians
        joints: list of joint definitions
        tool_offset: [x, y, z] tool offset in meters

    Returns:
        T: 4x4 homogeneous transform (base_link to tool0)
    """
    assert len(joint_angles) == len(joints), \
        f"Expected {len(joints)} joint angles, got {len(joint_angles)}"

    T = np.eye(4)  # Base frame

    for i, (joint, angle) in enumerate(zip(joints, joint_angles)):
        T_i = compute_joint_transform(joint, angle)
        T = T @ T_i

    # Apply tool offset (fixed joint)
    T_tool = make_transform(np.eye(3), np.array(tool_offset))
    T = T @ T_tool

    return T

def compute_all_transforms(joint_angles, joints=MA2010_JOINTS, tool_offset=TOOL0_OFFSET):
    """Compute all intermediate transforms (for debugging)"""
    transforms = []
    T = np.eye(4)

    for joint, angle in zip(joints, joint_angles):
        T_i = compute_joint_transform(joint, angle)
        T = T @ T_i
        transforms.append(T.copy())

    # Tool
    T_tool = make_transform(np.eye(3), np.array(tool_offset))
    T = T @ T_tool
    transforms.append(T.copy())

    return transforms


# ============================================================================
# Test Configurations
# ============================================================================

TEST_CONFIGS = {
    "home":        [0, 0, 0, 0, 0, 0],
    "j1_90":       [pi/2, 0, 0, 0, 0, 0],
    "j1_neg90":    [-pi/2, 0, 0, 0, 0, 0],
    "j2_45":       [0, pi/4, 0, 0, 0, 0],
    "j2_90":       [0, pi/2, 0, 0, 0, 0],
    "j3_45":       [0, 0, pi/4, 0, 0, 0],
    "j3_neg45":    [0, 0, -pi/4, 0, 0, 0],
    "j4_90":       [0, 0, 0, pi/2, 0, 0],
    "j5_45":       [0, 0, 0, 0, pi/4, 0],
    "j6_90":       [0, 0, 0, 0, 0, pi/2],
    "j12_combo":   [pi/4, pi/4, 0, 0, 0, 0],
    "j123_combo":  [pi/6, pi/4, -pi/6, 0, 0, 0],
    "all_small":   [0.1, -0.2, 0.3, -0.1, 0.2, -0.3],
    "random_1":    [0.5, -0.3, 1.2, -0.7, 0.4, -1.1],
    "random_2":    [-1.0, 0.8, -0.5, 2.0, -1.5, 0.3],
}


# ============================================================================
# Main
# ============================================================================

def main():
    print("=" * 80)
    print("MA2010 URDF Forward Kinematics Verification")
    print("=" * 80)
    print()

    # First, verify home position manually
    print("--- Manual Verification at Home (all joints = 0) ---")
    print("Expected chain at home:")
    print("  base_link -> J1(0,0,0.505) -> J2(0.150,0,0) -> J3(0,0,0.760)")
    print("  -> J4(0,0,0.200) -> J5(1.082,0,0) -> J6(0,0,0) -> tool0(0.100,0,0)")
    print()
    print("  X: 0.150 + 1.082 + 0.100 = 1.332 m = 1332 mm")
    print("  Y: 0")
    print("  Z: 0.505 + 0.760 + 0.200 = 1.465 m = 1465 mm")
    print()

    home_T = compute_fk([0, 0, 0, 0, 0, 0])
    home_pos = home_T[:3, 3]
    home_rpy = rotation_to_rpy(home_T[:3, :3])

    print(f"Computed home position: ({home_pos[0]*1000:.3f}, {home_pos[1]*1000:.3f}, {home_pos[2]*1000:.3f}) mm")
    print(f"Computed home RPY: ({home_rpy[0]:.6f}, {home_rpy[1]:.6f}, {home_rpy[2]:.6f}) rad")
    print(f"Home rotation matrix:\n{home_T[:3,:3]}")

    assert abs(home_pos[0] - 1.332) < 0.001, f"Home X wrong: {home_pos[0]}"
    assert abs(home_pos[1] - 0.0) < 0.001, f"Home Y wrong: {home_pos[1]}"
    assert abs(home_pos[2] - 1.465) < 0.001, f"Home Z wrong: {home_pos[2]}"
    print("HOME POSITION VERIFIED OK!")
    print()

    # Show intermediate transforms at home
    print("--- Intermediate transforms at home ---")
    transforms = compute_all_transforms([0, 0, 0, 0, 0, 0])
    joint_names = ["J1", "J2", "J3", "J4", "J5", "J6", "tool0"]
    for name, T in zip(joint_names, transforms):
        pos = T[:3, 3]
        print(f"  {name}: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
    print()

    # Compute all test configs
    print("--- All Test Configurations ---")
    print(f"{'Config':<15} {'X(mm)':>10} {'Y(mm)':>10} {'Z(mm)':>10} {'Roll':>10} {'Pitch':>10} {'Yaw':>10}")
    print("-" * 80)

    results = {}
    for name, angles in TEST_CONFIGS.items():
        T = compute_fk(angles)
        pos = T[:3, 3] * 1000  # Convert to mm
        rpy = rotation_to_rpy(T[:3, :3])

        results[name] = {
            "angles_rad": angles,
            "position_mm": pos.tolist(),
            "rpy_rad": rpy.tolist(),
            "rotation_matrix": T[:3, :3].tolist(),
        }

        print(f"{name:<15} {pos[0]:>10.3f} {pos[1]:>10.3f} {pos[2]:>10.3f} "
              f"{rpy[0]:>10.6f} {rpy[1]:>10.6f} {rpy[2]:>10.6f}")

    print()

    # Verify manufacturer reach
    print("--- Manufacturer Reach Check ---")
    print("MA2010 spec: max reach = 2010mm from base Z-axis")
    # At full extension: J2=90deg forward, J3=0, J5=0
    # Arm reaches horizontally: X = 0.150 + 0.760 + 0.200 + 1.082 + 0.100 = 2.292m
    # But that's from J1 axis, and at J2=90, the arm geometry changes
    # More accurate: full horizontal extension
    # J2=0 (arm up), rotate J2 by 90 so arm goes horizontal
    full_ext_angles = [0, pi/2, 0, 0, 0, 0]
    T_ext = compute_fk(full_ext_angles)
    pos_ext = T_ext[:3, 3] * 1000
    reach_from_z = np.sqrt(pos_ext[0]**2 + pos_ext[1]**2)
    print(f"At J2=90deg: position = ({pos_ext[0]:.1f}, {pos_ext[1]:.1f}, {pos_ext[2]:.1f}) mm")
    print(f"Horizontal reach from Z-axis: {reach_from_z:.1f} mm")
    print()

    # Output C++ test data
    print("--- C++ Test Data (copy to test_urdf_fk.cpp) ---")
    print()
    print("struct FKTestCase {")
    print("    std::string name;")
    print("    JointAngles angles;")
    print("    Vector3d expectedPosition;  // mm")
    print("    Vector3d expectedRPY;        // rad")
    print("};")
    print()
    print("const std::vector<FKTestCase> FK_TEST_CASES = {")
    for name, data in results.items():
        a = data["angles_rad"]
        p = data["position_mm"]
        r = data["rpy_rad"]
        angles_str = ", ".join(f"{v:.10f}" for v in a)
        print(f'    {{"{name}", {{{{{angles_str}}}}},')
        print(f'     Vector3d({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}),')
        print(f'     Vector3d({r[0]:.10f}, {r[1]:.10f}, {r[2]:.10f})}},')
    print("};")
    print()

    # Save JSON for easy loading
    json_path = "tests/fk_expected_values.json"
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Results saved to {json_path}")
    print()

    # Cross-check with roboticstoolbox if available
    if "--roboticstoolbox" in sys.argv or "--rtb" in sys.argv:
        try:
            cross_check_roboticstoolbox(results)
        except ImportError:
            print("roboticstoolbox-python not installed. Install with:")
            print("  pip install roboticstoolbox-python")

    print("=" * 80)
    print("VERIFICATION COMPLETE")
    print("=" * 80)


def cross_check_roboticstoolbox(manual_results):
    """Cross-check with roboticstoolbox-python (Peter Corke)"""
    import roboticstoolbox as rtb
    from spatialmath import SE3

    print("--- Cross-check with roboticstoolbox-python ---")

    # Build robot from URDF
    # Note: RTB can load URDF directly
    try:
        robot = rtb.Robot.URDF("tests/robots/ma2010.urdf")
        print(f"Loaded robot: {robot.name}, {robot.n} DOF")
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        print("Trying to build from DH instead...")
        return

    max_pos_err = 0.0
    max_rpy_err = 0.0

    for name, data in manual_results.items():
        angles = data["angles_rad"]
        expected_pos = np.array(data["position_mm"])
        expected_rpy = np.array(data["rpy_rad"])

        # RTB FK
        T_rtb = robot.fkine(angles)
        rtb_pos = np.array(T_rtb.t) * 1000  # m -> mm
        rtb_rpy = np.array(T_rtb.rpy(order='zyx'))  # RPY in radians

        pos_err = np.linalg.norm(rtb_pos - expected_pos)
        rpy_err = np.linalg.norm(rtb_rpy - expected_rpy)

        max_pos_err = max(max_pos_err, pos_err)
        max_rpy_err = max(max_rpy_err, rpy_err)

        status = "OK" if pos_err < 0.01 and rpy_err < 0.001 else "FAIL"
        if status == "FAIL":
            print(f"  {name}: pos_err={pos_err:.6f}mm, rpy_err={rpy_err:.6f}rad [{status}]")
            print(f"    Manual: pos=({expected_pos[0]:.3f}, {expected_pos[1]:.3f}, {expected_pos[2]:.3f})")
            print(f"    RTB:    pos=({rtb_pos[0]:.3f}, {rtb_pos[1]:.3f}, {rtb_pos[2]:.3f})")

    print(f"Max position error: {max_pos_err:.6f} mm")
    print(f"Max RPY error: {max_rpy_err:.6f} rad")

    if max_pos_err < 0.01:
        print("CROSS-CHECK PASSED: position < 0.01mm")
    else:
        print("CROSS-CHECK FAILED: position error too large!")

    if max_rpy_err < 0.001:
        print("CROSS-CHECK PASSED: RPY < 0.001 rad")
    else:
        print("CROSS-CHECK FAILED: RPY error too large!")
    print()


if __name__ == "__main__":
    main()
