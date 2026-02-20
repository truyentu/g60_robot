using System;
using System.Collections.Generic;

namespace RobotController.UI.Models;

/// <summary>
/// A single waypoint in a 5-DOF weld path.
/// Position (XYZ) + torch direction (work/travel angles) are task constraints.
/// SpinAngle is the redundant DOF to optimize for singularity avoidance.
/// </summary>
public class WeldPathPoint
{
    // Position (mm)
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    // Torch orientation (degrees)
    public double WorkAngle { get; set; }
    public double TravelAngle { get; set; }

    // Redundant DOF — spin angle around torch axis (degrees)
    public double SpinAngle { get; set; }

    // Diagnostics (filled after IK evaluation)
    public double? ManipulabilityIndex { get; set; }
    public double? PSIndex { get; set; }

    // Reference back to program point
    public string PointName { get; set; } = "";

    // Full 6-DOF joint solution (filled after IK)
    public double[]? JointAngles { get; set; }
}

/// <summary>
/// A sequence of weld path points forming a continuous weld seam.
/// </summary>
public class WeldPath
{
    public List<WeldPathPoint> Points { get; set; } = new();
    public double WeldSpeed { get; set; } // mm/s
    public string ProgramName { get; set; } = "";
}

/// <summary>
/// Result of MPSO spin angle optimization.
/// </summary>
public class SpinAngleOptimizationResult
{
    public double[] OptimizedSpinAngles { get; set; } = Array.Empty<double>();
    public double InitialCost { get; set; }
    public double FinalCost { get; set; }
    public double ImprovementPercent => InitialCost > 0
        ? (1.0 - FinalCost / InitialCost) * 100.0
        : 0.0;
    public int Iterations { get; set; }
    public double MinTheta5Before { get; set; }  // degrees
    public double MinTheta5After { get; set; }   // degrees
    public TimeSpan ElapsedTime { get; set; }
}
