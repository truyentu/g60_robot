using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace RobotController.UI.Models;

public enum ObjectCategory
{
    Other,
    Pedestal,
    WeldingTable,
    Fixture,
    Workpiece,
    Obstacle
}

/// <summary>
/// An imported 3D object in the scene (fixture, workpiece, obstacle, etc.)
/// </summary>
public class SceneObject
{
    public string Id { get; set; } = string.Empty;
    public string Name { get; set; } = string.Empty;
    public string MeshPath { get; set; } = string.Empty;

    // Position in mm (world coords if no parent, frame-local if parented)
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    // Rotation in degrees
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }

    // Scale factor (1.0 = original units, 1000.0 = meters→mm)
    public double Scale { get; set; } = 1.0;

    // Parent base frame ID (null = world)
    public string? ParentFrameId { get; set; }

    // Object category
    public ObjectCategory Category { get; set; } = ObjectCategory.Other;

    // WPF 3D geometry (runtime only, not serialized)
    public GeometryModel3D? Geometry { get; set; }

    // WPF 3D visual wrapper (runtime only, for gizmo binding)
    public ModelVisual3D? Visual { get; set; }

    // Material color
    public Color Color { get; set; } = Color.FromRgb(160, 160, 160);
}
