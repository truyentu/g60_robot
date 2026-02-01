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
