using System.Collections.Generic;

namespace RobotController.UI.Models;

/// <summary>
/// Parsed URDF robot structure
/// </summary>
public class UrdfRobot
{
    /// <summary>Robot name from URDF</summary>
    public string Name { get; set; } = "";

    /// <summary>All links in the robot</summary>
    public List<UrdfLink> Links { get; set; } = new();

    /// <summary>All joints in the robot</summary>
    public List<UrdfJoint> Joints { get; set; } = new();

    /// <summary>Source folder path (for resolving mesh paths)</summary>
    public string SourcePath { get; set; } = "";

    /// <summary>Original URDF/XACRO file path</summary>
    public string SourceFile { get; set; } = "";

    /// <summary>Get only revolute/prismatic joints (skip fixed)</summary>
    public List<UrdfJoint> ActiveJoints => Joints.FindAll(j => j.Type is "revolute" or "prismatic");

    /// <summary>Number of degrees of freedom</summary>
    public int DOF => ActiveJoints.Count;
}

/// <summary>
/// URDF link element (contains mesh info)
/// </summary>
public class UrdfLink
{
    /// <summary>Link name</summary>
    public string Name { get; set; } = "";

    /// <summary>Visual mesh absolute path (resolved from package://)</summary>
    public string? VisualMeshPath { get; set; }

    /// <summary>Collision mesh absolute path</summary>
    public string? CollisionMeshPath { get; set; }

    /// <summary>Visual origin XYZ in mm (converted from meters)</summary>
    public double[] VisualOrigin { get; set; } = { 0, 0, 0 };

    /// <summary>Visual origin RPY in radians</summary>
    public double[] VisualRpy { get; set; } = { 0, 0, 0 };

    /// <summary>Check if link has visual mesh</summary>
    public bool HasVisualMesh => !string.IsNullOrEmpty(VisualMeshPath);
}

/// <summary>
/// URDF joint element (contains transform and limits)
/// </summary>
public class UrdfJoint
{
    /// <summary>Joint name</summary>
    public string Name { get; set; } = "";

    /// <summary>Joint type: revolute, prismatic, fixed, continuous</summary>
    public string Type { get; set; } = "revolute";

    /// <summary>Parent link name</summary>
    public string ParentLink { get; set; } = "";

    /// <summary>Child link name</summary>
    public string ChildLink { get; set; } = "";

    /// <summary>Joint origin XYZ in mm (converted from meters)</summary>
    public double[] OriginXyz { get; set; } = { 0, 0, 0 };

    /// <summary>Joint origin RPY in radians</summary>
    public double[] OriginRpy { get; set; } = { 0, 0, 0 };

    /// <summary>Rotation/translation axis [x, y, z]</summary>
    public double[] Axis { get; set; } = { 0, 0, 1 };

    /// <summary>Lower limit in degrees (for revolute) or mm (for prismatic)</summary>
    public double LimitLower { get; set; } = -180;

    /// <summary>Upper limit in degrees or mm</summary>
    public double LimitUpper { get; set; } = 180;

    /// <summary>Max velocity in deg/s or mm/s</summary>
    public double VelocityMax { get; set; } = 180;

    /// <summary>Max effort (torque/force)</summary>
    public double EffortMax { get; set; } = 0;

    /// <summary>Check if joint is movable (not fixed)</summary>
    public bool IsMovable => Type is "revolute" or "prismatic" or "continuous";
}

/// <summary>
/// Result of URDF parsing with validation info
/// </summary>
public class UrdfParseResult
{
    /// <summary>Parsed robot (null if failed)</summary>
    public UrdfRobot? Robot { get; set; }

    /// <summary>Whether parsing succeeded</summary>
    public bool Success { get; set; }

    /// <summary>Error message if failed</summary>
    public string? ErrorMessage { get; set; }

    /// <summary>Warning messages (non-fatal issues)</summary>
    public List<string> Warnings { get; set; } = new();

    /// <summary>Info messages (for user feedback)</summary>
    public List<string> InfoMessages { get; set; } = new();

    /// <summary>List of mesh files found</summary>
    public List<string> MeshFilesFound { get; set; } = new();

    /// <summary>List of mesh files missing</summary>
    public List<string> MeshFilesMissing { get; set; } = new();
}
