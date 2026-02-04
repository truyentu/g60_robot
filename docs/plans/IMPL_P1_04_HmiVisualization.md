# IMPL_P1_04: HMI & 3D Visualization

| Metadata      | Value                           |
|---------------|---------------------------------|
| Plan ID       | IMPL_P1_04                      |
| Covers Tasks  | P1-11, P1-12, P1-13, P1-14, P1-15 |
| Status        | DRAFT                           |
| Version       | 1.0                             |
| Created       | 2026-02-01                      |
| Prerequisites | IMPL_P1_01, IMPL_P1_02, IMPL_P1_03 completed |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết kế HMI Robot KUKA WPF.md` | MVVM pattern, UI layout, control design |
| P0 | `ressearch_doc_md/Thiết Kế Mô Phỏng Robot WPF Helix.md` | Helix3D setup, viewport, robot model loading |

---

## Prerequisites

Trước khi bắt đầu, đảm bảo:

| Requirement | Check |
|-------------|-------|
| IMPL_P1_01 completed | Project structure, HelixToolkit installed |
| IMPL_P1_02 completed | IPC layer working |
| IMPL_P1_03 completed | Config system working |
| Robot STL files | 3D models for robot links |

### Verification

```powershell
# Check HelixToolkit is installed
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"
dotnet list RobotController.UI package | Select-String "Helix"

# Expected:
# > HelixToolkit.Wpf    2.24.0
```

---

## Overview

### HMI Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           MAIN WINDOW                                    │
├─────────────────────────────────────────────────────────────────────────┤
│ Menu Bar: File | Robot | View | Help                                    │
├─────────────────────────────────────────────────────────────────────────┤
│ Toolbar: [Connect] [Disconnect] [Refresh] | [Home] | [Run] [Stop]       │
├────────────────┬────────────────────────────────────┬───────────────────┤
│                │                                    │                   │
│  Navigation    │         3D VIEWPORT                │   Properties      │
│  Panel         │                                    │   Panel           │
│  ┌──────────┐  │   ┌─────────────────────────────┐  │  ┌─────────────┐  │
│  │ Manual   │  │   │                             │  │  │ Status      │  │
│  │ Program  │  │   │      Robot 3D Model         │  │  │ ─────────── │  │
│  │ I/O      │  │   │                             │  │  │ State: IDLE │  │
│  │ Config   │  │   │         [Robot]             │  │  │             │  │
│  │ Diag     │  │   │           /\                │  │  │ Joints      │  │
│  └──────────┘  │   │          /  \               │  │  │ ─────────── │  │
│                │   │         /    \              │  │  │ J1: 0.0°    │  │
│                │   │                             │  │  │ J2: -45.0°  │  │
│                │   │    Grid + Axes              │  │  │ ...         │  │
│                │   │                             │  │  │             │  │
│                │   └─────────────────────────────┘  │  │ TCP         │  │
│                │                                    │  │ ─────────── │  │
│                │   Camera: Orbit | Pan | Zoom       │  │ X: 500mm    │  │
│                │                                    │  └─────────────┘  │
├────────────────┴────────────────────────────────────┴───────────────────┤
│ Status Bar: [●] Connected | IDLE | Robot Controller v1.0.0              │
└─────────────────────────────────────────────────────────────────────────┘
```

### Robot Visualization

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       ROBOT 3D VISUALIZATION                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   STL Files (7 links):              Transform Hierarchy:                 │
│   ┌──────────────┐                  ┌──────────────────────────────────┐│
│   │ base.stl     │                  │ World                            ││
│   │ link1.stl    │                  │  └─ Base (fixed)                 ││
│   │ link2.stl    │                  │      └─ Link1 (J1 rotation)      ││
│   │ link3.stl    │                  │          └─ Link2 (J2 rotation)  ││
│   │ link4.stl    │                  │              └─ Link3 (J3 rot)   ││
│   │ link5.stl    │                  │                  └─ Link4 (J4)   ││
│   │ link6.stl    │                  │                      └─ Link5    ││
│   └──────────────┘                  │                          └─ Link6││
│                                     │                              └─TCP││
│                                     └──────────────────────────────────┘│
│   Forward Kinematics:                                                    │
│   T_0_1 = Rz(θ1) · Tz(d1) · Tx(a1) · Rx(α1)                            │
│   T_0_n = T_0_1 · T_1_2 · ... · T_(n-1)_n                               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## PART A: Robot STL Models (P1-13 Preparation)

### Step 1: Create Placeholder STL Files

Nếu chưa có STL files thực, tạo placeholder models.

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

# Ensure resources/models directory exists
New-Item -ItemType Directory -Force -Path "resources\models"

Write-Host "[OK] Models directory ready" -ForegroundColor Green
Write-Host "[INFO] Place robot STL files in resources/models/" -ForegroundColor Yellow
Write-Host "  Required files:" -ForegroundColor Yellow
Write-Host "  - base.stl (or link0.stl)" -ForegroundColor Yellow
Write-Host "  - link1.stl through link6.stl" -ForegroundColor Yellow
```

**Note:** Nếu không có STL files, plan sẽ sử dụng generated geometry (boxes/cylinders).

---

### Step 2: Create STL Generator Script (nếu cần test)

**File:** `scripts/generate_test_robot.py`

```powershell
$stlGeneratorPy = @"
"""
Generate simple robot STL files for testing
Requires: numpy-stl (pip install numpy-stl)
"""

import numpy as np
from stl import mesh
import os

def create_box(width, height, depth):
    """Create a box mesh centered at origin"""
    vertices = np.array([
        [-width/2, -height/2, -depth/2],
        [+width/2, -height/2, -depth/2],
        [+width/2, +height/2, -depth/2],
        [-width/2, +height/2, -depth/2],
        [-width/2, -height/2, +depth/2],
        [+width/2, -height/2, +depth/2],
        [+width/2, +height/2, +depth/2],
        [-width/2, +height/2, +depth/2],
    ])

    faces = np.array([
        [0,3,1], [1,3,2],  # front
        [0,4,7], [0,7,3],  # left
        [4,5,6], [4,6,7],  # back
        [5,1,2], [5,2,6],  # right
        [2,3,6], [3,7,6],  # top
        [0,1,5], [0,5,4],  # bottom
    ])

    box = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            box.vectors[i][j] = vertices[f[j], :]
    return box

def create_cylinder(radius, height, segments=32):
    """Create a cylinder mesh along Z axis"""
    angles = np.linspace(0, 2*np.pi, segments, endpoint=False)

    # Create vertices
    bottom = np.array([[radius*np.cos(a), radius*np.sin(a), 0] for a in angles])
    top = np.array([[radius*np.cos(a), radius*np.sin(a), height] for a in angles])
    center_bottom = np.array([0, 0, 0])
    center_top = np.array([0, 0, height])

    faces = []
    vertices_list = []

    # Side faces
    for i in range(segments):
        next_i = (i + 1) % segments
        # Two triangles per side segment
        vertices_list.extend([bottom[i], bottom[next_i], top[i]])
        vertices_list.extend([top[i], bottom[next_i], top[next_i]])

    # Bottom cap
    for i in range(segments):
        next_i = (i + 1) % segments
        vertices_list.extend([center_bottom, bottom[next_i], bottom[i]])

    # Top cap
    for i in range(segments):
        next_i = (i + 1) % segments
        vertices_list.extend([center_top, top[i], top[next_i]])

    cyl = mesh.Mesh(np.zeros(len(vertices_list)//3, dtype=mesh.Mesh.dtype))
    for i in range(0, len(vertices_list), 3):
        idx = i // 3
        cyl.vectors[idx] = vertices_list[i:i+3]

    return cyl

def main():
    output_dir = "../resources/models"
    os.makedirs(output_dir, exist_ok=True)

    print("Generating test robot STL files...")

    # Base - large cylinder
    base = create_cylinder(150, 50, 32)
    base.save(f"{output_dir}/base.stl")
    print("  Created base.stl")

    # Link 1 - cylinder (rotation around Z)
    link1 = create_cylinder(80, 300, 24)
    link1.save(f"{output_dir}/link1.stl")
    print("  Created link1.stl")

    # Link 2 - box (shoulder)
    link2 = create_box(100, 560, 80)
    link2.save(f"{output_dir}/link2.stl")
    print("  Created link2.stl")

    # Link 3 - box (elbow)
    link3 = create_box(80, 80, 80)
    link3.save(f"{output_dir}/link3.stl")
    print("  Created link3.stl")

    # Link 4 - cylinder (wrist 1)
    link4 = create_cylinder(50, 400, 20)
    link4.save(f"{output_dir}/link4.stl")
    print("  Created link4.stl")

    # Link 5 - box (wrist 2)
    link5 = create_box(60, 60, 60)
    link5.save(f"{output_dir}/link5.stl")
    print("  Created link5.stl")

    # Link 6 - small cylinder (tool flange)
    link6 = create_cylinder(40, 80, 16)
    link6.save(f"{output_dir}/link6.stl")
    print("  Created link6.stl")

    print(f"Done! Files saved to {output_dir}/")

if __name__ == "__main__":
    main()
"@

Set-Content -Path "scripts\generate_test_robot.py" -Value $stlGeneratorPy -Encoding UTF8
Write-Host "[OK] STL generator script created" -ForegroundColor Green
Write-Host "[INFO] Run: pip install numpy-stl && python scripts/generate_test_robot.py" -ForegroundColor Yellow
```

---

## PART B: Robot Model Classes (P1-13)

### Step 3: Create RobotLink Model

**File:** `src/ui/RobotController.UI/Models/RobotLink.cs`

```powershell
# Create Models directory if needed
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\Models"

$robotLinkCs = @"
using System.Windows.Media.Media3D;

namespace RobotController.UI.Models;

/// <summary>
/// DH Parameter for a robot joint
/// </summary>
public class DHParameter
{
    /// <summary>Joint index (1-6)</summary>
    public int Joint { get; set; }

    /// <summary>Link length (mm)</summary>
    public double A { get; set; }

    /// <summary>Link twist (degrees)</summary>
    public double Alpha { get; set; }

    /// <summary>Link offset (mm)</summary>
    public double D { get; set; }

    /// <summary>Joint angle offset (degrees)</summary>
    public double ThetaOffset { get; set; }

    /// <summary>Alpha in radians</summary>
    public double AlphaRad => Alpha * Math.PI / 180.0;

    /// <summary>ThetaOffset in radians</summary>
    public double ThetaOffsetRad => ThetaOffset * Math.PI / 180.0;
}

/// <summary>
/// Represents a single robot link with its mesh and transform
/// </summary>
public class RobotLink
{
    /// <summary>Link name (e.g., "Link1", "Link2")</summary>
    public string Name { get; set; } = string.Empty;

    /// <summary>Joint index (0 for base, 1-6 for joints)</summary>
    public int JointIndex { get; set; }

    /// <summary>DH Parameters for this joint</summary>
    public DHParameter? DH { get; set; }

    /// <summary>3D geometry for this link</summary>
    public GeometryModel3D? Geometry { get; set; }

    /// <summary>Visual representation in the scene</summary>
    public ModelVisual3D? Visual { get; set; }

    /// <summary>Current joint angle (radians)</summary>
    public double JointAngle { get; set; }

    /// <summary>Transform from previous link to this link</summary>
    public Matrix3D LocalTransform { get; set; } = Matrix3D.Identity;

    /// <summary>Transform from world to this link</summary>
    public Matrix3D WorldTransform { get; set; } = Matrix3D.Identity;

    /// <summary>Path to STL file</summary>
    public string? StlPath { get; set; }

    /// <summary>
    /// Calculate DH transform matrix for given joint angle
    /// </summary>
    /// <param name="theta">Joint angle in radians</param>
    /// <returns>4x4 transformation matrix</returns>
    public Matrix3D CalculateDHTransform(double theta)
    {
        if (DH == null) return Matrix3D.Identity;

        double a = DH.A;
        double d = DH.D;
        double alpha = DH.AlphaRad;
        double thetaTotal = theta + DH.ThetaOffsetRad;

        double ct = Math.Cos(thetaTotal);
        double st = Math.Sin(thetaTotal);
        double ca = Math.Cos(alpha);
        double sa = Math.Sin(alpha);

        // Standard DH transformation matrix
        // T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        return new Matrix3D(
            ct,      -st * ca,   st * sa,   0,
            st,       ct * ca,  -ct * sa,   0,
            0,        sa,        ca,        0,
            a * ct,   a * st,    d,         1
        );
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Models\RobotLink.cs" -Value $robotLinkCs -Encoding UTF8
Write-Host "[OK] RobotLink.cs created" -ForegroundColor Green
```

---

### Step 4: Create RobotModel3D Class

**File:** `src/ui/RobotController.UI/Models/RobotModel3D.cs`

```powershell
$robotModel3DCs = @"
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.Common.Config;
using Serilog;

namespace RobotController.UI.Models;

/// <summary>
/// 3D Robot Model with Forward Kinematics
/// </summary>
public class RobotModel3D
{
    private readonly List<RobotLink> _links = new();
    private readonly double[] _jointAngles = new double[6];
    private Model3DGroup? _modelGroup;

    /// <summary>Robot name from config</summary>
    public string Name { get; private set; } = "Robot";

    /// <summary>Number of joints (always 6)</summary>
    public int NumJoints => 6;

    /// <summary>All robot links including base</summary>
    public IReadOnlyList<RobotLink> Links => _links;

    /// <summary>Current joint angles in degrees</summary>
    public double[] JointAnglesDegrees => _jointAngles.Select(a => a * 180.0 / Math.PI).ToArray();

    /// <summary>3D model group for viewport</summary>
    public Model3DGroup? ModelGroup => _modelGroup;

    /// <summary>TCP (Tool Center Point) position</summary>
    public Point3D TcpPosition { get; private set; }

    /// <summary>TCP orientation as rotation matrix</summary>
    public Matrix3D TcpOrientation { get; private set; } = Matrix3D.Identity;

    /// <summary>
    /// Initialize robot model from configuration
    /// </summary>
    public bool Initialize(RobotConfigData config, string modelsPath)
    {
        try
        {
            Name = config.Name;
            _links.Clear();
            _modelGroup = new Model3DGroup();

            Log.Information("Initializing robot model: {Name}", Name);

            // Create base link (fixed)
            var baseLink = CreateLink(0, "Base", null, modelsPath);
            if (baseLink != null)
            {
                _links.Add(baseLink);
                if (baseLink.Geometry != null)
                    _modelGroup.Children.Add(baseLink.Geometry);
            }

            // Create joint links (1-6)
            for (int i = 0; i < config.DHParameters.Count && i < 6; i++)
            {
                var dhParam = config.DHParameters[i];
                var link = CreateLink(i + 1, $"Link{i + 1}", dhParam, modelsPath);
                if (link != null)
                {
                    _links.Add(link);
                    if (link.Geometry != null)
                        _modelGroup.Children.Add(link.Geometry);
                }
            }

            // Set home position
            if (config.HomePosition.Length >= 6)
            {
                for (int i = 0; i < 6; i++)
                {
                    _jointAngles[i] = config.HomePosition[i] * Math.PI / 180.0;
                }
            }

            // Initial FK update
            UpdateForwardKinematics();

            Log.Information("Robot model initialized with {Count} links", _links.Count);
            return true;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to initialize robot model");
            return false;
        }
    }

    /// <summary>
    /// Create a robot link with geometry
    /// </summary>
    private RobotLink? CreateLink(int jointIndex, string name, DHParameterData? dhData, string modelsPath)
    {
        var link = new RobotLink
        {
            Name = name,
            JointIndex = jointIndex
        };

        // Set DH parameters
        if (dhData != null)
        {
            link.DH = new DHParameter
            {
                Joint = dhData.Joint,
                A = dhData.A,
                Alpha = dhData.Alpha,
                D = dhData.D,
                ThetaOffset = dhData.ThetaOffset
            };
        }

        // Try to load STL
        string stlPath = Path.Combine(modelsPath, $"{name.ToLower()}.stl");
        if (!File.Exists(stlPath))
        {
            stlPath = Path.Combine(modelsPath, $"link{jointIndex}.stl");
        }

        GeometryModel3D? geometry = null;

        if (File.Exists(stlPath))
        {
            geometry = LoadStlGeometry(stlPath, GetLinkColor(jointIndex));
            link.StlPath = stlPath;
            Log.Debug("Loaded STL for {Name}: {Path}", name, stlPath);
        }
        else
        {
            // Create placeholder geometry
            geometry = CreatePlaceholderGeometry(jointIndex);
            Log.Debug("Created placeholder geometry for {Name}", name);
        }

        link.Geometry = geometry;
        return link;
    }

    /// <summary>
    /// Load geometry from STL file
    /// </summary>
    private GeometryModel3D? LoadStlGeometry(string path, Color color)
    {
        try
        {
            var reader = new StLReader();
            var model = reader.Read(path);

            if (model.Children.Count > 0 && model.Children[0] is GeometryModel3D geo)
            {
                geo.Material = new DiffuseMaterial(new SolidColorBrush(color));
                geo.BackMaterial = new DiffuseMaterial(new SolidColorBrush(color));
                return geo;
            }

            return null;
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "Failed to load STL: {Path}", path);
            return null;
        }
    }

    /// <summary>
    /// Create placeholder geometry when STL not available
    /// </summary>
    private GeometryModel3D CreatePlaceholderGeometry(int jointIndex)
    {
        var builder = new MeshBuilder();
        var color = GetLinkColor(jointIndex);

        switch (jointIndex)
        {
            case 0: // Base
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 50), 100, 32);
                break;
            case 1: // Link 1 (rotation around Z)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 200), 60, 24);
                break;
            case 2: // Link 2 (shoulder)
                builder.AddBox(new Point3D(0, 0, 280), 60, 60, 560);
                break;
            case 3: // Link 3 (elbow)
                builder.AddBox(new Point3D(0, 0, 0), 50, 50, 50);
                break;
            case 4: // Link 4 (wrist 1)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 300), 40, 20);
                break;
            case 5: // Link 5 (wrist 2)
                builder.AddBox(new Point3D(0, 0, 0), 40, 40, 40);
                break;
            case 6: // Link 6 (flange)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 60), 30, 16);
                break;
            default:
                builder.AddBox(new Point3D(0, 0, 0), 30, 30, 30);
                break;
        }

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(color));

        return new GeometryModel3D(mesh, material)
        {
            BackMaterial = material
        };
    }

    /// <summary>
    /// Get color for each link
    /// </summary>
    private Color GetLinkColor(int jointIndex)
    {
        return jointIndex switch
        {
            0 => Color.FromRgb(80, 80, 80),      // Base: Dark gray
            1 => Color.FromRgb(255, 140, 0),    // Link1: Orange
            2 => Color.FromRgb(255, 165, 0),    // Link2: Light orange
            3 => Color.FromRgb(255, 180, 0),    // Link3: Yellow-orange
            4 => Color.FromRgb(100, 149, 237),  // Link4: Cornflower blue
            5 => Color.FromRgb(65, 105, 225),   // Link5: Royal blue
            6 => Color.FromRgb(50, 50, 50),     // Link6: Dark gray (flange)
            _ => Color.FromRgb(128, 128, 128)   // Default: Gray
        };
    }

    /// <summary>
    /// Update a single joint angle (degrees)
    /// </summary>
    public void SetJointAngle(int jointIndex, double angleDegrees)
    {
        if (jointIndex < 1 || jointIndex > 6) return;
        _jointAngles[jointIndex - 1] = angleDegrees * Math.PI / 180.0;
        UpdateForwardKinematics();
    }

    /// <summary>
    /// Update all joint angles (degrees)
    /// </summary>
    public void SetAllJointAngles(double[] anglesDegrees)
    {
        if (anglesDegrees.Length < 6) return;
        for (int i = 0; i < 6; i++)
        {
            _jointAngles[i] = anglesDegrees[i] * Math.PI / 180.0;
        }
        UpdateForwardKinematics();
    }

    /// <summary>
    /// Update forward kinematics - recalculate all transforms
    /// </summary>
    public void UpdateForwardKinematics()
    {
        Matrix3D worldTransform = Matrix3D.Identity;

        for (int i = 0; i < _links.Count; i++)
        {
            var link = _links[i];

            if (link.JointIndex == 0)
            {
                // Base is fixed
                link.WorldTransform = Matrix3D.Identity;
            }
            else
            {
                // Calculate DH transform for this joint
                int jointIdx = link.JointIndex - 1;
                double theta = jointIdx < _jointAngles.Length ? _jointAngles[jointIdx] : 0;

                Matrix3D dhTransform = link.CalculateDHTransform(theta);
                worldTransform = Matrix3D.Multiply(dhTransform, worldTransform);
                link.WorldTransform = worldTransform;
            }

            // Apply transform to geometry
            if (link.Geometry != null)
            {
                link.Geometry.Transform = new MatrixTransform3D(link.WorldTransform);
            }
        }

        // Update TCP position (end of last link)
        if (_links.Count > 0)
        {
            var lastLink = _links[^1];
            TcpPosition = lastLink.WorldTransform.Transform(new Point3D(0, 0, 0));
            TcpOrientation = lastLink.WorldTransform;
        }
    }

    /// <summary>
    /// Get TCP position as array [X, Y, Z, Rx, Ry, Rz]
    /// </summary>
    public double[] GetTcpPose()
    {
        // Extract position
        double x = TcpPosition.X;
        double y = TcpPosition.Y;
        double z = TcpPosition.Z;

        // Extract Euler angles (simplified - ZYX convention)
        // Note: Full implementation would use proper rotation matrix decomposition
        double rx = 0, ry = 0, rz = 0;

        if (TcpOrientation.IsAffine)
        {
            // Simplified extraction - for demo purposes
            // Real implementation should use proper matrix to Euler conversion
            rz = Math.Atan2(TcpOrientation.M21, TcpOrientation.M11) * 180 / Math.PI;
            ry = Math.Atan2(-TcpOrientation.M31,
                Math.Sqrt(TcpOrientation.M32 * TcpOrientation.M32 + TcpOrientation.M33 * TcpOrientation.M33)) * 180 / Math.PI;
            rx = Math.Atan2(TcpOrientation.M32, TcpOrientation.M33) * 180 / Math.PI;
        }

        return new[] { x, y, z, rx, ry, rz };
    }
}

/// <summary>
/// Robot config data from Core (matches JSON structure)
/// </summary>
public class RobotConfigData
{
    public string Name { get; set; } = "Robot";
    public string Model { get; set; } = "";
    public List<DHParameterData> DHParameters { get; set; } = new();
    public double[] HomePosition { get; set; } = new double[6];
}

public class DHParameterData
{
    public int Joint { get; set; }
    public double A { get; set; }
    public double Alpha { get; set; }
    public double D { get; set; }
    public double ThetaOffset { get; set; }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Models\RobotModel3D.cs" -Value $robotModel3DCs -Encoding UTF8
Write-Host "[OK] RobotModel3D.cs created" -ForegroundColor Green
```

---

## PART C: 3D Viewport Service (P1-12)

### Step 5: Create Viewport Service

**File:** `src/ui/RobotController.UI/Services/ViewportService.cs`

```powershell
$viewportServiceCs = @"
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.Common.Config;
using RobotController.UI.Models;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for 3D viewport service
/// </summary>
public interface IViewportService
{
    /// <summary>Robot 3D model</summary>
    RobotModel3D? Robot { get; }

    /// <summary>Is robot model loaded</summary>
    bool IsLoaded { get; }

    /// <summary>Initialize viewport with robot model</summary>
    Task<bool> InitializeAsync(RobotConfigData config);

    /// <summary>Update joint angles from status</summary>
    void UpdateJointAngles(double[] anglesDegrees);

    /// <summary>Get the 3D model group for binding</summary>
    Model3DGroup? GetModelGroup();

    /// <summary>Create default lighting</summary>
    Model3DGroup CreateLighting();

    /// <summary>Create grid</summary>
    Model3DGroup CreateGrid(double size = 2000, double spacing = 100);

    /// <summary>Create coordinate axes</summary>
    Model3DGroup CreateAxes(double length = 200);

    /// <summary>Create TCP marker</summary>
    Model3DGroup CreateTcpMarker();

    /// <summary>Event when robot model updates</summary>
    event EventHandler? ModelUpdated;
}

/// <summary>
/// 3D Viewport service implementation
/// </summary>
public class ViewportService : IViewportService
{
    private RobotModel3D? _robot;
    private readonly string _modelsPath;
    private Model3DGroup? _tcpMarker;

    public RobotModel3D? Robot => _robot;
    public bool IsLoaded => _robot != null;

    public event EventHandler? ModelUpdated;

    public ViewportService()
    {
        // Path to models relative to executable
        _modelsPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory,
            "..", "..", "..", "..", "..", "resources", "models"
        );
    }

    public async Task<bool> InitializeAsync(RobotConfigData config)
    {
        return await Task.Run(() =>
        {
            try
            {
                Log.Information("Initializing viewport with robot: {Name}", config.Name);

                _robot = new RobotModel3D();
                bool success = _robot.Initialize(config, _modelsPath);

                if (success)
                {
                    ModelUpdated?.Invoke(this, EventArgs.Empty);
                }

                return success;
            }
            catch (Exception ex)
            {
                Log.Error(ex, "Failed to initialize viewport");
                return false;
            }
        });
    }

    public void UpdateJointAngles(double[] anglesDegrees)
    {
        if (_robot == null) return;

        _robot.SetAllJointAngles(anglesDegrees);

        // Update TCP marker position
        if (_tcpMarker != null)
        {
            var tcpPos = _robot.TcpPosition;
            _tcpMarker.Transform = new TranslateTransform3D(tcpPos.X, tcpPos.Y, tcpPos.Z);
        }

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public Model3DGroup? GetModelGroup()
    {
        return _robot?.ModelGroup;
    }

    public Model3DGroup CreateLighting()
    {
        var group = new Model3DGroup();

        // Ambient light
        group.Children.Add(new AmbientLight(Color.FromRgb(60, 60, 60)));

        // Main directional light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(200, 200, 200),
            new Vector3D(-1, -1, -1)));

        // Fill light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(100, 100, 100),
            new Vector3D(1, 0.5, 0.5)));

        // Back light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(80, 80, 80),
            new Vector3D(0, 1, 1)));

        return group;
    }

    public Model3DGroup CreateGrid(double size = 2000, double spacing = 100)
    {
        var group = new Model3DGroup();
        var builder = new MeshBuilder();

        double halfSize = size / 2;
        double lineThickness = 1;

        var gridColor = Color.FromRgb(60, 60, 60);
        var majorColor = Color.FromRgb(80, 80, 80);

        // Create grid lines along X
        for (double y = -halfSize; y <= halfSize; y += spacing)
        {
            bool isMajor = Math.Abs(y) < 0.001 || Math.Abs(y % (spacing * 5)) < 0.001;
            builder.AddPipe(
                new Point3D(-halfSize, y, 0),
                new Point3D(halfSize, y, 0),
                0, isMajor ? lineThickness * 2 : lineThickness, 8);
        }

        // Create grid lines along Y
        for (double x = -halfSize; x <= halfSize; x += spacing)
        {
            bool isMajor = Math.Abs(x) < 0.001 || Math.Abs(x % (spacing * 5)) < 0.001;
            builder.AddPipe(
                new Point3D(x, -halfSize, 0),
                new Point3D(x, halfSize, 0),
                0, isMajor ? lineThickness * 2 : lineThickness, 8);
        }

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(gridColor));
        group.Children.Add(new GeometryModel3D(mesh, material));

        return group;
    }

    public Model3DGroup CreateAxes(double length = 200)
    {
        var group = new Model3DGroup();
        double radius = length / 40;
        double arrowLength = length / 5;

        // X axis - Red
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(length, 0, 0),
            radius, arrowLength, Colors.Red);

        // Y axis - Green
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, length, 0),
            radius, arrowLength, Colors.LimeGreen);

        // Z axis - Blue
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, length),
            radius, arrowLength, Colors.DodgerBlue);

        return group;
    }

    private void AddAxis(Model3DGroup group, Point3D start, Point3D end,
        double radius, double arrowLength, Color color)
    {
        var builder = new MeshBuilder();

        // Shaft
        var direction = end - start;
        direction.Normalize();
        var shaftEnd = end - direction * arrowLength;
        builder.AddCylinder(start, shaftEnd, radius, 16);

        // Arrow head
        builder.AddCone(shaftEnd, direction, radius * 2.5, radius * 2.5, arrowLength, true, true, 16);

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });
    }

    public Model3DGroup CreateTcpMarker()
    {
        var group = new Model3DGroup();
        var builder = new MeshBuilder();

        // Small sphere at TCP
        builder.AddSphere(new Point3D(0, 0, 0), 15, 16, 16);

        // Small coordinate frame
        double axisLength = 50;
        double axisRadius = 2;

        // X
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(axisLength, 0, 0), axisRadius, 8);
        // Y
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, axisLength, 0), axisRadius, 8);
        // Z
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, axisLength), axisRadius, 8);

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(Colors.Magenta));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });

        _tcpMarker = group;
        return group;
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Services\ViewportService.cs" -Value $viewportServiceCs -Encoding UTF8
Write-Host "[OK] ViewportService.cs created" -ForegroundColor Green
```

---

## PART D: Enhanced MainWindow (P1-11)

### Step 6: Update MainWindow.xaml

**File:** `src/ui/RobotController.UI/Views/MainWindow.xaml`

```powershell
$mainWindowXaml = @"
<Window x:Class="RobotController.UI.Views.MainWindow"
        xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
        xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
        xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
        xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
        xmlns:helix="http://helix-toolkit.org/wpf"
        xmlns:vm="clr-namespace:RobotController.UI.ViewModels"
        mc:Ignorable="d"
        Title="Robot Controller"
        Height="800" Width="1400"
        MinHeight="600" MinWidth="1000"
        Background="{StaticResource PrimaryBackground}"
        WindowStartupLocation="CenterScreen">

    <Window.Resources>
        <!-- Button Style -->
        <Style TargetType="Button" x:Key="ToolbarButton">
            <Setter Property="Background" Value="Transparent"/>
            <Setter Property="Foreground" Value="{StaticResource ForegroundBrush}"/>
            <Setter Property="BorderThickness" Value="0"/>
            <Setter Property="Padding" Value="10,5"/>
            <Setter Property="Margin" Value="2"/>
            <Style.Triggers>
                <Trigger Property="IsMouseOver" Value="True">
                    <Setter Property="Background" Value="#3F3F46"/>
                </Trigger>
                <Trigger Property="IsEnabled" Value="False">
                    <Setter Property="Foreground" Value="#666666"/>
                </Trigger>
            </Style.Triggers>
        </Style>

        <!-- Slider Style -->
        <Style TargetType="Slider" x:Key="JogSlider">
            <Setter Property="Minimum" Value="-180"/>
            <Setter Property="Maximum" Value="180"/>
            <Setter Property="Width" Value="150"/>
            <Setter Property="Margin" Value="5,2"/>
        </Style>
    </Window.Resources>

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>  <!-- Menu -->
            <RowDefinition Height="Auto"/>  <!-- Toolbar -->
            <RowDefinition Height="*"/>     <!-- Content -->
            <RowDefinition Height="Auto"/>  <!-- Status Bar -->
        </Grid.RowDefinitions>

        <!-- Menu Bar -->
        <Menu Grid.Row="0" Background="{StaticResource SecondaryBackground}"
              Foreground="{StaticResource ForegroundBrush}">
            <MenuItem Header="_File">
                <MenuItem Header="_New Program" InputGestureText="Ctrl+N"/>
                <MenuItem Header="_Open Program" InputGestureText="Ctrl+O"/>
                <MenuItem Header="_Save Program" InputGestureText="Ctrl+S"/>
                <Separator/>
                <MenuItem Header="E_xit" Click="MenuItem_Exit_Click" InputGestureText="Alt+F4"/>
            </MenuItem>
            <MenuItem Header="_Robot">
                <MenuItem Header="_Connect" Command="{Binding ConnectCommand}"/>
                <MenuItem Header="_Disconnect" Command="{Binding DisconnectCommand}"/>
                <Separator/>
                <MenuItem Header="_Home All Axes" IsEnabled="False"/>
                <MenuItem Header="_Enable Motors" IsEnabled="False"/>
            </MenuItem>
            <MenuItem Header="_View">
                <MenuItem Header="Reset _Camera" Click="MenuItem_ResetCamera_Click"/>
                <MenuItem Header="_Top View" Click="MenuItem_TopView_Click"/>
                <MenuItem Header="_Front View" Click="MenuItem_FrontView_Click"/>
                <MenuItem Header="_Side View" Click="MenuItem_SideView_Click"/>
                <Separator/>
                <MenuItem Header="Show _Grid" IsCheckable="True" IsChecked="{Binding ShowGrid}"/>
                <MenuItem Header="Show _Axes" IsCheckable="True" IsChecked="{Binding ShowAxes}"/>
                <MenuItem Header="Show _TCP" IsCheckable="True" IsChecked="{Binding ShowTcp}"/>
            </MenuItem>
            <MenuItem Header="_Help">
                <MenuItem Header="_About" Click="MenuItem_About_Click"/>
            </MenuItem>
        </Menu>

        <!-- Toolbar -->
        <Border Grid.Row="1" Background="{StaticResource SecondaryBackground}"
                BorderBrush="{StaticResource BorderBrush}" BorderThickness="0,0,0,1">
            <StackPanel Orientation="Horizontal" Margin="5,3">
                <Button Content="Connect" Command="{Binding ConnectCommand}" Style="{StaticResource ToolbarButton}"/>
                <Button Content="Disconnect" Command="{Binding DisconnectCommand}" Style="{StaticResource ToolbarButton}"/>
                <Separator Style="{StaticResource {x:Static ToolBar.SeparatorStyleKey}}" Margin="5,0"/>
                <Button Content="Refresh" Command="{Binding RefreshStatusCommand}" Style="{StaticResource ToolbarButton}"/>
                <Separator Style="{StaticResource {x:Static ToolBar.SeparatorStyleKey}}" Margin="5,0"/>
                <Button Content="Home" Style="{StaticResource ToolbarButton}" IsEnabled="False"/>
                <Separator Style="{StaticResource {x:Static ToolBar.SeparatorStyleKey}}" Margin="5,0"/>
                <Button Content="Run" Style="{StaticResource ToolbarButton}" IsEnabled="False" Foreground="LimeGreen"/>
                <Button Content="Pause" Style="{StaticResource ToolbarButton}" IsEnabled="False" Foreground="Orange"/>
                <Button Content="Stop" Style="{StaticResource ToolbarButton}" IsEnabled="False" Foreground="Red"/>
                <Separator Style="{StaticResource {x:Static ToolBar.SeparatorStyleKey}}" Margin="5,0"/>
                <TextBlock Text="Speed:" VerticalAlignment="Center" Foreground="Gray" Margin="5,0"/>
                <Slider Value="{Binding SpeedOverride}" Minimum="0" Maximum="100" Width="100"
                        VerticalAlignment="Center"/>
                <TextBlock Text="{Binding SpeedOverride, StringFormat={}{0:F0}%}" Width="35"
                           VerticalAlignment="Center" Foreground="{StaticResource ForegroundBrush}"/>
            </StackPanel>
        </Border>

        <!-- Main Content -->
        <Grid Grid.Row="2">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="200" MinWidth="150"/>
                <ColumnDefinition Width="Auto"/>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="Auto"/>
                <ColumnDefinition Width="260" MinWidth="200"/>
            </Grid.ColumnDefinitions>

            <!-- Navigation Panel -->
            <Border Grid.Column="0" Background="{StaticResource SecondaryBackground}">
                <Grid>
                    <Grid.RowDefinitions>
                        <RowDefinition Height="Auto"/>
                        <RowDefinition Height="*"/>
                    </Grid.RowDefinitions>

                    <TextBlock Grid.Row="0" Text="Navigation" FontWeight="Bold" Margin="10"
                               Foreground="{StaticResource ForegroundBrush}"/>

                    <ListBox Grid.Row="1" Background="Transparent" BorderThickness="0"
                             Foreground="{StaticResource ForegroundBrush}"
                             SelectedIndex="{Binding SelectedNavIndex}">
                        <ListBox.ItemContainerStyle>
                            <Style TargetType="ListBoxItem">
                                <Setter Property="Padding" Value="10,8"/>
                                <Style.Triggers>
                                    <Trigger Property="IsSelected" Value="True">
                                        <Setter Property="Background" Value="{StaticResource AccentBrush}"/>
                                    </Trigger>
                                </Style.Triggers>
                            </Style>
                        </ListBox.ItemContainerStyle>
                        <ListBoxItem Content="Manual Jog"/>
                        <ListBoxItem Content="Program"/>
                        <ListBoxItem Content="I/O"/>
                        <ListBoxItem Content="Configuration"/>
                        <ListBoxItem Content="Diagnostics"/>
                    </ListBox>
                </Grid>
            </Border>

            <!-- Left Splitter -->
            <GridSplitter Grid.Column="1" Width="3" Background="{StaticResource BorderBrush}"
                          HorizontalAlignment="Center" VerticalAlignment="Stretch"/>

            <!-- 3D Viewport -->
            <Border Grid.Column="2" Background="#1A1A1A">
                <helix:HelixViewport3D x:Name="Viewport3D"
                                       Background="#1E1E1E"
                                       ShowCoordinateSystem="True"
                                       CoordinateSystemLabelForeground="White"
                                       ZoomExtentsWhenLoaded="True"
                                       CameraRotationMode="Turntable"
                                       IsRotationEnabled="True"
                                       IsPanEnabled="True"
                                       IsZoomEnabled="True"
                                       ShowViewCube="True"
                                       ViewCubeBackText="Back"
                                       ViewCubeFrontText="Front"
                                       ViewCubeLeftText="Left"
                                       ViewCubeRightText="Right"
                                       ViewCubeTopText="Top"
                                       ViewCubeBottomText="Bottom">

                    <!-- Camera -->
                    <helix:HelixViewport3D.Camera>
                        <PerspectiveCamera Position="1500,1500,1000"
                                           LookDirection="-1,-1,-0.5"
                                           UpDirection="0,0,1"
                                           FieldOfView="45"/>
                    </helix:HelixViewport3D.Camera>

                    <!-- Lighting -->
                    <ModelVisual3D x:Name="LightingModel">
                        <ModelVisual3D.Content>
                            <Model3DGroup>
                                <AmbientLight Color="#404040"/>
                                <DirectionalLight Color="#C8C8C8" Direction="-1,-1,-1"/>
                                <DirectionalLight Color="#646464" Direction="1,0.5,0.5"/>
                            </Model3DGroup>
                        </ModelVisual3D.Content>
                    </ModelVisual3D>

                    <!-- Grid -->
                    <helix:GridLinesVisual3D x:Name="GridLines"
                                             Width="2000" Length="2000"
                                             MajorDistance="500" MinorDistance="100"
                                             Thickness="1"
                                             Fill="#404040"
                                             Visible="{Binding ShowGrid}"/>

                    <!-- Coordinate Axes -->
                    <helix:CoordinateSystemVisual3D x:Name="CoordAxes"
                                                    ArrowLengths="200"
                                                    Visible="{Binding ShowAxes}"/>

                    <!-- Robot Model Container -->
                    <ModelVisual3D x:Name="RobotModelVisual"/>

                    <!-- TCP Marker Container -->
                    <ModelVisual3D x:Name="TcpMarkerVisual"/>

                </helix:HelixViewport3D>
            </Border>

            <!-- Right Splitter -->
            <GridSplitter Grid.Column="3" Width="3" Background="{StaticResource BorderBrush}"
                          HorizontalAlignment="Center" VerticalAlignment="Stretch"/>

            <!-- Properties Panel -->
            <Border Grid.Column="4" Background="{StaticResource SecondaryBackground}">
                <ScrollViewer VerticalScrollBarVisibility="Auto">
                    <StackPanel Margin="10">
                        <!-- Connection Status -->
                        <TextBlock Text="Connection" FontWeight="Bold" Margin="0,0,0,5"
                                   Foreground="{StaticResource ForegroundBrush}"/>
                        <Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
                            <Grid>
                                <Grid.ColumnDefinitions>
                                    <ColumnDefinition Width="*"/>
                                    <ColumnDefinition Width="Auto"/>
                                </Grid.ColumnDefinitions>
                                <StackPanel>
                                    <TextBlock Text="{Binding ConnectionStatus}"
                                               Foreground="{Binding ConnectionStatusColor}"
                                               FontWeight="Bold" FontSize="14"/>
                                    <TextBlock Text="{Binding CoreVersion, StringFormat=Core v{0}}"
                                               Foreground="Gray" FontSize="10"
                                               Visibility="{Binding IsConnected, Converter={StaticResource BoolToVisibility}}"/>
                                </StackPanel>
                                <Ellipse Grid.Column="1" Width="12" Height="12"
                                         Fill="{Binding ConnectionStatusColor}"
                                         VerticalAlignment="Center"/>
                            </Grid>
                        </Border>

                        <!-- Robot Status -->
                        <TextBlock Text="Robot Status" FontWeight="Bold" Margin="0,0,0,5"
                                   Foreground="{StaticResource ForegroundBrush}"/>
                        <Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
                            <Grid>
                                <Grid.RowDefinitions>
                                    <RowDefinition/>
                                    <RowDefinition/>
                                    <RowDefinition/>
                                </Grid.RowDefinitions>
                                <Grid.ColumnDefinitions>
                                    <ColumnDefinition Width="*"/>
                                    <ColumnDefinition Width="*"/>
                                </Grid.ColumnDefinitions>

                                <StackPanel Grid.Row="0" Grid.Column="0">
                                    <TextBlock Text="State" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding RobotState}" FontWeight="Bold"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="0" Grid.Column="1">
                                    <TextBlock Text="Mode" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding RobotMode}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="1" Grid.Column="0" Margin="0,8,0,0">
                                    <TextBlock Text="Homed" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding IsHomed}"
                                               Foreground="{Binding IsHomed, Converter={StaticResource BoolToColor}}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="1" Grid.Column="1" Margin="0,8,0,0">
                                    <TextBlock Text="Enabled" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding IsEnabled}"
                                               Foreground="{Binding IsEnabled, Converter={StaticResource BoolToColor}}"/>
                                </StackPanel>
                            </Grid>
                        </Border>

                        <!-- Joint Positions -->
                        <TextBlock Text="Joint Positions" FontWeight="Bold" Margin="0,0,0,5"
                                   Foreground="{StaticResource ForegroundBrush}"/>
                        <Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
                            <ItemsControl ItemsSource="{Binding JointPositions}">
                                <ItemsControl.ItemTemplate>
                                    <DataTemplate>
                                        <Grid Margin="0,3">
                                            <Grid.ColumnDefinitions>
                                                <ColumnDefinition Width="35"/>
                                                <ColumnDefinition Width="*"/>
                                                <ColumnDefinition Width="70"/>
                                            </Grid.ColumnDefinitions>
                                            <TextBlock Grid.Column="0" Text="{Binding Name}"
                                                       Foreground="Gray" FontWeight="Bold"/>
                                            <ProgressBar Grid.Column="1"
                                                         Value="{Binding Value}" Minimum="-180" Maximum="180"
                                                         Height="6" Margin="5,0"
                                                         Background="#222222" Foreground="{StaticResource AccentBrush}"/>
                                            <TextBlock Grid.Column="2" Text="{Binding Value, StringFormat={}{0:F2}°}"
                                                       Foreground="{StaticResource ForegroundBrush}"
                                                       HorizontalAlignment="Right"/>
                                        </Grid>
                                    </DataTemplate>
                                </ItemsControl.ItemTemplate>
                            </ItemsControl>
                        </Border>

                        <!-- TCP Position -->
                        <TextBlock Text="TCP Position" FontWeight="Bold" Margin="0,0,0,5"
                                   Foreground="{StaticResource ForegroundBrush}"/>
                        <Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
                            <Grid>
                                <Grid.RowDefinitions>
                                    <RowDefinition/>
                                    <RowDefinition/>
                                </Grid.RowDefinitions>
                                <Grid.ColumnDefinitions>
                                    <ColumnDefinition/>
                                    <ColumnDefinition/>
                                    <ColumnDefinition/>
                                </Grid.ColumnDefinitions>

                                <!-- Position -->
                                <StackPanel Grid.Row="0" Grid.Column="0" Margin="0,0,5,5">
                                    <TextBlock Text="X" Foreground="Red" FontSize="10" FontWeight="Bold"/>
                                    <TextBlock Text="{Binding TcpX, StringFormat={}{0:F1}}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="0" Grid.Column="1" Margin="0,0,5,5">
                                    <TextBlock Text="Y" Foreground="LimeGreen" FontSize="10" FontWeight="Bold"/>
                                    <TextBlock Text="{Binding TcpY, StringFormat={}{0:F1}}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="0" Grid.Column="2" Margin="0,0,0,5">
                                    <TextBlock Text="Z" Foreground="DodgerBlue" FontSize="10" FontWeight="Bold"/>
                                    <TextBlock Text="{Binding TcpZ, StringFormat={}{0:F1}}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>

                                <!-- Orientation -->
                                <StackPanel Grid.Row="1" Grid.Column="0" Margin="0,0,5,0">
                                    <TextBlock Text="Rx" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding TcpRx, StringFormat={}{0:F1}°}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="1" Grid.Column="1" Margin="0,0,5,0">
                                    <TextBlock Text="Ry" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding TcpRy, StringFormat={}{0:F1}°}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                                <StackPanel Grid.Row="1" Grid.Column="2">
                                    <TextBlock Text="Rz" Foreground="Gray" FontSize="10"/>
                                    <TextBlock Text="{Binding TcpRz, StringFormat={}{0:F1}°}"
                                               Foreground="{StaticResource ForegroundBrush}"/>
                                </StackPanel>
                            </Grid>
                        </Border>

                        <!-- Manual Jog (simple version) -->
                        <TextBlock Text="Manual Jog" FontWeight="Bold" Margin="0,0,0,5"
                                   Foreground="{StaticResource ForegroundBrush}"/>
                        <Border Background="#333333" CornerRadius="3" Padding="10" Margin="0,0,0,15">
                            <StackPanel>
                                <TextBlock Text="(Coming in Phase 2)" Foreground="Gray"
                                           HorizontalAlignment="Center"/>
                            </StackPanel>
                        </Border>
                    </StackPanel>
                </ScrollViewer>
            </Border>
        </Grid>

        <!-- Status Bar -->
        <Border Grid.Row="3" Background="{StaticResource SecondaryBackground}"
                BorderBrush="{StaticResource BorderBrush}" BorderThickness="0,1,0,0">
            <Grid Margin="5,3">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0" Orientation="Horizontal">
                    <Ellipse Width="10" Height="10" Fill="{Binding ConnectionStatusColor}" Margin="0,0,5,0"/>
                    <TextBlock Text="{Binding ConnectionStatus}" Foreground="{StaticResource ForegroundBrush}"/>
                </StackPanel>

                <TextBlock Grid.Column="1" Text="|" Foreground="Gray" Margin="10,0"/>

                <TextBlock Grid.Column="2" Text="{Binding RobotState}" Foreground="{StaticResource ForegroundBrush}"/>

                <TextBlock Grid.Column="3" Text="Robot Controller v1.0.0" Foreground="Gray"/>
            </Grid>
        </Border>
    </Grid>
</Window>
"@

Set-Content -Path "src\ui\RobotController.UI\Views\MainWindow.xaml" -Value $mainWindowXaml -Encoding UTF8
Write-Host "[OK] MainWindow.xaml updated" -ForegroundColor Green
```

---

### Step 7: Update MainWindow.xaml.cs

**File:** `src/ui/RobotController.UI/Views/MainWindow.xaml.cs`

```powershell
$mainWindowCs = @"
using System.Windows;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using Serilog;

namespace RobotController.UI.Views;

public partial class MainWindow : Window
{
    private MainViewModel? _viewModel;

    public MainWindow()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        if (e.NewValue is MainViewModel vm)
        {
            _viewModel = vm;
            _viewModel.RobotModelUpdated += OnRobotModelUpdated;
            _viewModel.PropertyChanged += OnViewModelPropertyChanged;
        }
    }

    private void OnRobotModelUpdated(object? sender, EventArgs e)
    {
        Dispatcher.Invoke(() =>
        {
            if (_viewModel?.RobotModelGroup != null)
            {
                RobotModelVisual.Content = _viewModel.RobotModelGroup;
                Log.Debug("Robot model updated in viewport");
            }

            if (_viewModel?.TcpMarkerGroup != null)
            {
                TcpMarkerVisual.Content = _viewModel.TcpMarkerGroup;
            }
        });
    }

    private void OnViewModelPropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
    {
        // Handle property changes if needed
    }

    private void MenuItem_Exit_Click(object sender, RoutedEventArgs e)
    {
        Application.Current.Shutdown();
    }

    private void MenuItem_ResetCamera_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(1500, 1500, 1000);
        Viewport3D.Camera.LookDirection = new Vector3D(-1, -1, -0.5);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_TopView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(0, 0, 2500);
        Viewport3D.Camera.LookDirection = new Vector3D(0, 0, -1);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 1, 0);
    }

    private void MenuItem_FrontView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(0, -2500, 500);
        Viewport3D.Camera.LookDirection = new Vector3D(0, 1, 0);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_SideView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(2500, 0, 500);
        Viewport3D.Camera.LookDirection = new Vector3D(-1, 0, 0);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_About_Click(object sender, RoutedEventArgs e)
    {
        MessageBox.Show(
            "Robot Controller v1.0.0\n\n" +
            "6-DOF Robot Controller for MIG/MAG Welding\n\n" +
            "© 2026 All Rights Reserved",
            "About Robot Controller",
            MessageBoxButton.OK,
            MessageBoxImage.Information);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Views\MainWindow.xaml.cs" -Value $mainWindowCs -Encoding UTF8
Write-Host "[OK] MainWindow.xaml.cs updated" -ForegroundColor Green
```

---

### Step 8: Add Value Converters

**File:** `src/ui/RobotController.UI/Converters/Converters.cs`

```powershell
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\Converters"

$convertersCs = @"
using System.Globalization;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;

namespace RobotController.UI.Converters;

/// <summary>
/// Converts bool to Visibility
/// </summary>
public class BoolToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return b ? Visibility.Visible : Visibility.Collapsed;
        }
        return Visibility.Collapsed;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts bool to color (green for true, gray for false)
/// </summary>
public class BoolToColorConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return b ? Brushes.LimeGreen : Brushes.Gray;
        }
        return Brushes.Gray;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Inverts bool value
/// </summary>
public class InverseBoolConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return !b;
        }
        return false;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return !b;
        }
        return false;
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Converters\Converters.cs" -Value $convertersCs -Encoding UTF8
Write-Host "[OK] Converters.cs created" -ForegroundColor Green
```

---

### Step 9: Update App.xaml with Converters

**File:** `src/ui/RobotController.UI/App.xaml`

```powershell
$appXaml = @"
<Application x:Class="RobotController.UI.App"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:converters="clr-namespace:RobotController.UI.Converters"
             StartupUri="Views/MainWindow.xaml">
    <Application.Resources>
        <!-- Converters -->
        <converters:BoolToVisibilityConverter x:Key="BoolToVisibility"/>
        <converters:BoolToColorConverter x:Key="BoolToColor"/>
        <converters:InverseBoolConverter x:Key="InverseBool"/>

        <!-- Colors -->
        <SolidColorBrush x:Key="PrimaryBackground" Color="#1E1E1E"/>
        <SolidColorBrush x:Key="SecondaryBackground" Color="#252526"/>
        <SolidColorBrush x:Key="BorderBrush" Color="#3F3F46"/>
        <SolidColorBrush x:Key="ForegroundBrush" Color="#CCCCCC"/>
        <SolidColorBrush x:Key="AccentBrush" Color="#007ACC"/>
    </Application.Resources>
</Application>
"@

Set-Content -Path "src\ui\RobotController.UI\App.xaml" -Value $appXaml -Encoding UTF8
Write-Host "[OK] App.xaml updated" -ForegroundColor Green
```

---

## PART E: Updated ViewModel (P1-14)

### Step 10: Update MainViewModel with Viewport Integration

**File:** `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`

```powershell
$mainViewModelCs = @"
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Config;
using RobotController.Common.Messages;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.Text.Json;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly IIpcClientService _ipcClient;
    private readonly IConfigService _configService;
    private readonly IViewportService _viewportService;
    private readonly Dispatcher _dispatcher;
    private bool _disposed;
    private bool _robotInitialized;

    // Connection
    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private bool _isConnected;

    // Robot Status
    [ObservableProperty]
    private string _robotState = "UNKNOWN";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    // TCP Position
    [ObservableProperty]
    private double _tcpX;

    [ObservableProperty]
    private double _tcpY;

    [ObservableProperty]
    private double _tcpZ;

    [ObservableProperty]
    private double _tcpRx;

    [ObservableProperty]
    private double _tcpRy;

    [ObservableProperty]
    private double _tcpRz;

    // Core Info
    [ObservableProperty]
    private string _coreVersion = "";

    [ObservableProperty]
    private long _coreUptime;

    [ObservableProperty]
    private string _robotName = "";

    [ObservableProperty]
    private string _robotModel = "";

    // UI State
    [ObservableProperty]
    private int _selectedNavIndex = 0;

    [ObservableProperty]
    private double _speedOverride = 100;

    [ObservableProperty]
    private bool _showGrid = true;

    [ObservableProperty]
    private bool _showAxes = true;

    [ObservableProperty]
    private bool _showTcp = true;

    // 3D Model
    public Model3DGroup? RobotModelGroup => _viewportService.GetModelGroup();
    public Model3DGroup? TcpMarkerGroup { get; private set; }

    public event EventHandler? RobotModelUpdated;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = 0.0 },
        new JointPosition { Name = "J3", Value = 0.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 0.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    /// <summary>
    /// Design-time constructor
    /// </summary>
    public MainViewModel()
        : this(new IpcClientService(), new ConfigService(), new ViewportService())
    {
    }

    /// <summary>
    /// Runtime constructor with DI
    /// </summary>
    public MainViewModel(IIpcClientService ipcClient, IConfigService configService, IViewportService viewportService)
    {
        _ipcClient = ipcClient;
        _configService = configService;
        _viewportService = viewportService;
        _dispatcher = Dispatcher.CurrentDispatcher;

        // Subscribe to IPC events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
        _ipcClient.ErrorOccurred += OnErrorOccurred;

        // Subscribe to viewport events
        _viewportService.ModelUpdated += OnViewportModelUpdated;

        // Create TCP marker
        TcpMarkerGroup = _viewportService.CreateTcpMarker();

        // Auto-connect if configured
        if (_configService.Config.Connection.AutoConnect)
        {
            _ = ConnectAsync();
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (_ipcClient.IsConnected)
        {
            Log.Information("Already connected");
            return;
        }

        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        var connConfig = _configService.Config.Connection;
        Log.Information("Connecting to {Address}...", connConfig.RepAddress);

        bool success = await _ipcClient.ConnectAsync(
            connConfig.RepAddress,
            connConfig.SubAddress
        );

        if (success)
        {
            // Get version info
            var pong = await _ipcClient.PingAsync();
            if (pong != null)
            {
                CoreVersion = pong.CoreVersion;
                CoreUptime = pong.UptimeMs;
            }

            // Get robot config and initialize 3D model
            await InitializeRobotModelAsync();

            // Get initial status
            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }
        }
    }

    private async Task InitializeRobotModelAsync()
    {
        if (_robotInitialized) return;

        try
        {
            // Request robot config from Core
            var request = new IpcMessage
            {
                Type = MessageTypes.GET_CONFIG,
                Id = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
            };

            var response = await _ipcClient.SendRequestAsync(request);
            if (response != null && response.Type == MessageTypes.CONFIG)
            {
                var configJson = response.Payload.GetRawText();
                var robotConfig = JsonSerializer.Deserialize<RobotConfigData>(configJson,
                    new JsonSerializerOptions { PropertyNameCaseInsensitive = true });

                if (robotConfig != null)
                {
                    RobotName = robotConfig.Name;
                    RobotModel = robotConfig.Model;

                    // Initialize 3D model
                    bool modelOk = await _viewportService.InitializeAsync(robotConfig);
                    if (modelOk)
                    {
                        _robotInitialized = true;
                        _dispatcher.Invoke(() =>
                        {
                            OnPropertyChanged(nameof(RobotModelGroup));
                            RobotModelUpdated?.Invoke(this, EventArgs.Empty);
                        });
                        Log.Information("Robot 3D model initialized: {Name}", robotConfig.Name);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to initialize robot model");
        }
    }

    [RelayCommand]
    private void Disconnect()
    {
        _ipcClient.Disconnect();
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (!_ipcClient.IsConnected)
        {
            return;
        }

        var status = await _ipcClient.GetStatusAsync();
        if (status != null)
        {
            UpdateStatus(status);
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        _dispatcher.InvokeAsync(() => UpdateStatus(status));
    }

    private void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        _dispatcher.InvokeAsync(() =>
        {
            IsConnected = isConnected;
            if (isConnected)
            {
                ConnectionStatus = "Connected";
                ConnectionStatusColor = Brushes.LimeGreen;
            }
            else
            {
                ConnectionStatus = "Disconnected";
                ConnectionStatusColor = Brushes.Red;
                RobotState = "UNKNOWN";
            }
        });
    }

    private void OnErrorOccurred(object? sender, string error)
    {
        Log.Error("IPC Error: {Error}", error);
    }

    private void OnViewportModelUpdated(object? sender, EventArgs e)
    {
        _dispatcher.InvokeAsync(() =>
        {
            OnPropertyChanged(nameof(RobotModelGroup));
            RobotModelUpdated?.Invoke(this, EventArgs.Empty);
        });
    }

    private void UpdateStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        // Update joint positions
        if (status.Joints.Length >= 6)
        {
            double[] angles = new double[6];
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
                angles[i] = status.Joints[i];
            }
            OnPropertyChanged(nameof(JointPositions));

            // Update 3D model
            _viewportService.UpdateJointAngles(angles);
        }

        // Update TCP position
        if (status.TcpPosition.Length >= 6)
        {
            TcpX = status.TcpPosition[0];
            TcpY = status.TcpPosition[1];
            TcpZ = status.TcpPosition[2];
            TcpRx = status.TcpPosition[3];
            TcpRy = status.TcpPosition[4];
            TcpRz = status.TcpPosition[5];
        }
    }

    public void Dispose()
    {
        if (_disposed) return;

        _ipcClient.StatusReceived -= OnStatusReceived;
        _ipcClient.ConnectionStateChanged -= OnConnectionStateChanged;
        _ipcClient.ErrorOccurred -= OnErrorOccurred;
        _viewportService.ModelUpdated -= OnViewportModelUpdated;
        _ipcClient.Dispose();

        _disposed = true;
        GC.SuppressFinalize(this);
    }
}

public class JointPosition : ObservableObject
{
    private string _name = string.Empty;
    private double _value;

    public string Name
    {
        get => _name;
        set => SetProperty(ref _name, value);
    }

    public double Value
    {
        get => _value;
        set => SetProperty(ref _value, value);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\ViewModels\MainViewModel.cs" -Value $mainViewModelCs -Encoding UTF8
Write-Host "[OK] MainViewModel.cs updated with viewport integration" -ForegroundColor Green
```

---

### Step 11: Update App.xaml.cs for DI

**File:** `src/ui/RobotController.UI/App.xaml.cs`

```powershell
$appXamlCs = @"
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using RobotController.UI.Views;
using Serilog;
using System.Windows;

namespace RobotController.UI;

public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup Serilog
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("../../logs/ui.log",
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7,
                outputTemplate: "{Timestamp:yyyy-MM-dd HH:mm:ss.fff} [{Level:u3}] {Message:lj}{NewLine}{Exception}")
            .CreateLogger();

        Log.Information("========================================");
        Log.Information("Robot Controller UI starting...");
        Log.Information("========================================");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // Configuration
                services.AddSingleton<IConfigService, ConfigService>();

                // Services
                services.AddSingleton<IIpcClientService, IpcClientService>();
                services.AddSingleton<IViewportService, ViewportService>();

                // ViewModels
                services.AddSingleton<MainViewModel>();

                // Views
                services.AddSingleton<MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Get services
        var configService = _host.Services.GetRequiredService<IConfigService>();
        var mainWindow = _host.Services.GetRequiredService<MainWindow>();
        var viewModel = _host.Services.GetRequiredService<MainViewModel>();

        // Apply window settings
        var windowConfig = configService.Config.Window;
        if (windowConfig.Width > 0) mainWindow.Width = windowConfig.Width;
        if (windowConfig.Height > 0) mainWindow.Height = windowConfig.Height;
        if (windowConfig.Left >= 0) mainWindow.Left = windowConfig.Left;
        if (windowConfig.Top >= 0) mainWindow.Top = windowConfig.Top;
        if (windowConfig.Maximized) mainWindow.WindowState = WindowState.Maximized;

        mainWindow.DataContext = viewModel;
        mainWindow.Closing += MainWindow_Closing;
        mainWindow.Show();

        Log.Information("UI initialized");
    }

    private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
    {
        if (sender is MainWindow window && _host != null)
        {
            var configService = _host.Services.GetService<IConfigService>();
            if (configService != null)
            {
                configService.Config.Window.Width = (int)window.Width;
                configService.Config.Window.Height = (int)window.Height;
                configService.Config.Window.Left = (int)window.Left;
                configService.Config.Window.Top = (int)window.Top;
                configService.Config.Window.Maximized = window.WindowState == WindowState.Maximized;
                configService.Save();
            }
        }
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        if (_host != null)
        {
            var viewModel = _host.Services.GetService<MainViewModel>();
            viewModel?.Dispose();

            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\App.xaml.cs" -Value $appXamlCs -Encoding UTF8
Write-Host "[OK] App.xaml.cs updated" -ForegroundColor Green
```

---

## PART F: Build and Test (P1-15)

### Step 12: Build Solution

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"

# Clean and rebuild
dotnet clean
dotnet restore
dotnet build --configuration Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] C# build failed" -ForegroundColor Red
    exit 1
}

Write-Host "[OK] C# UI built successfully" -ForegroundColor Green
```

---

### Step 13: Integration Test

#### 13.1 Start C++ Core

```powershell
# Terminal 1
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"
.\build\bin\Release\robot_core.exe
```

#### 13.2 Start C# UI

```powershell
# Terminal 2
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"
dotnet run --project RobotController.UI --configuration Release
```

---

### Step 14: Verification Checklist

| Test | Expected | Pass |
|------|----------|------|
| **Window Layout** | | |
| Menu bar visible | File, Robot, View, Help | [ ] |
| Toolbar visible | Connect, Disconnect, Refresh, etc. | [ ] |
| Navigation panel | 5 items visible | [ ] |
| 3D viewport | Grid and axes visible | [ ] |
| Properties panel | Status, Joints, TCP sections | [ ] |
| Status bar | Connection status, state | [ ] |
| **3D Viewport** | | |
| Grid displayed | 2000x2000mm grid | [ ] |
| Axes displayed | RGB for XYZ | [ ] |
| View cube | Top right corner | [ ] |
| Camera orbit | Left mouse drag | [ ] |
| Camera pan | Middle mouse drag | [ ] |
| Camera zoom | Scroll wheel | [ ] |
| View buttons | Reset, Top, Front, Side work | [ ] |
| **Connection** | | |
| Auto-connect | Connects on startup | [ ] |
| Status indicator | Green when connected | [ ] |
| Core version | Shows in properties | [ ] |
| Disconnect | Status turns red | [ ] |
| Reconnect | Works after disconnect | [ ] |
| **Robot Model** | | |
| Model loads | Robot visible in viewport | [ ] |
| Joint values | Update from status | [ ] |
| TCP position | Shows X, Y, Z, Rx, Ry, Rz | [ ] |
| FK works | Model moves with joint values | [ ] |
| **Integration** | | |
| Status updates | 10 Hz from Core | [ ] |
| No lag | UI responsive | [ ] |
| Memory stable | No leaks | [ ] |
| Clean shutdown | Both processes exit cleanly | [ ] |

---

### Step 15: Git Commit

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

git add .
git commit -m "IMPL_P1_04: HMI & 3D Visualization complete

HMI Shell (P1-11):
- Complete MainWindow layout with dark theme
- Menu bar, toolbar, navigation panel
- Properties panel with status, joints, TCP
- Status bar with connection indicator

3D Viewport (P1-12):
- HelixToolkit viewport with grid and axes
- View cube for orientation
- Camera orbit/pan/zoom controls
- Preset views (Top, Front, Side, Reset)

Robot Model (P1-13):
- RobotLink and RobotModel3D classes
- STL loading support
- Placeholder geometry when STL not available
- DH parameter integration

3D Visualization (P1-14):
- Forward Kinematics implementation
- Real-time joint angle updates
- TCP marker
- ViewportService for model management

Integration (P1-15):
- Full integration test passed
- IPC status updates at 10 Hz
- Smooth 3D visualization

Tasks completed: P1-11, P1-12, P1-13, P1-14, P1-15

Phase 1 COMPLETE - \"Hello Robot\" milestone achieved!

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Completion Checklist

| Item | Status |
|------|--------|
| RobotLink.cs created | [ ] |
| RobotModel3D.cs created | [ ] |
| ViewportService.cs created | [ ] |
| MainWindow.xaml updated | [ ] |
| MainWindow.xaml.cs updated | [ ] |
| Converters created | [ ] |
| App.xaml updated | [ ] |
| MainViewModel updated | [ ] |
| App.xaml.cs updated | [ ] |
| C# builds successfully | [ ] |
| 3D viewport works | [ ] |
| Camera controls work | [ ] |
| Robot model displays | [ ] |
| FK updates work | [ ] |
| Status updates work | [ ] |
| Integration test passed | [ ] |
| Git commit created | [ ] |

---

## Troubleshooting

### Problem: HelixToolkit viewport blank

**Solution:**
- Check GPU drivers
- Try software rendering: `RenderOptions.ProcessRenderMode = RenderMode.SoftwareOnly;`

### Problem: Robot model not loading

**Solution:**
- Check STL files exist in `resources/models/`
- Check file permissions
- Placeholder geometry should appear if STL missing

### Problem: FK not updating

**Solution:**
- Check DH parameters in config
- Verify joint angles are in degrees
- Check Matrix3D multiplication order

### Problem: Camera controls not working

**Solution:**
- Ensure `IsRotationEnabled`, `IsPanEnabled`, `IsZoomEnabled` are True
- Check mouse events not blocked by overlay

---

## Phase 1 Complete

**Milestone: "Hello Robot" ACHIEVED**

- [x] C# UI khởi động thành công
- [x] Hiển thị mô hình robot 3D trong viewport
- [x] C++ Core process chạy độc lập
- [x] UI và Core giao tiếp được qua IPC
- [x] Config load được từ file YAML/JSON
- [x] Logs ghi được vào file và console

## Next Steps

1. Update IMPLEMENTATION_PLAN_TRACKER.md
2. Phase 1 Review
3. Begin Phase 2: Motion Core

---

*Document Version: 1.0 | Created: 2026-02-01*
