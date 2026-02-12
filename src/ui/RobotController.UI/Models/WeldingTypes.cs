namespace RobotController.UI.Models;

/// <summary>
/// Robot Target - Position + Orientation + Configuration.
/// Mirrors ABB 'robtarget' structure.
/// Format in code: CONST robtarget p1 := [[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[e1..e6]];
/// </summary>
public class RobTarget
{
    public string Name { get; set; } = "";

    // Position (mm)
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    // Orientation (Quaternion)
    public double Q1 { get; set; } = 1.0;
    public double Q2 { get; set; }
    public double Q3 { get; set; }
    public double Q4 { get; set; }

    // Configuration data
    public int Cf1 { get; set; }
    public int Cf4 { get; set; }
    public int Cf6 { get; set; }
    public int Cfx { get; set; }

    // External axes (9E9 = unused)
    public double[] ExtAx { get; set; } = { 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 };

    /// <summary>
    /// Serialize to RPL inline format: [[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[e1..e6]]
    /// </summary>
    public string ToRplString()
    {
        var extAxStr = string.Join(",", ExtAx.Select(e =>
            double.IsInfinity(e) || Math.Abs(e - 9E9) < 1 ? "9E9" : e.ToString("F2")));

        return $"[[{X:F2},{Y:F2},{Z:F2}]," +
               $"[{Q1:F4},{Q2:F4},{Q3:F4},{Q4:F4}]," +
               $"[{Cf1},{Cf4},{Cf6},{Cfx}]," +
               $"[{extAxStr}]]";
    }

    /// <summary>
    /// Create from current robot TCP pose [x,y,z,rx,ry,rz]
    /// Euler angles (degrees) are converted to quaternion.
    /// </summary>
    public static RobTarget FromPose(string name, double[] pose)
    {
        var rt = new RobTarget { Name = name };

        if (pose.Length >= 3)
        {
            rt.X = pose[0];
            rt.Y = pose[1];
            rt.Z = pose[2];
        }

        if (pose.Length >= 6)
        {
            // Convert Euler RPY (degrees) to Quaternion
            var (q1, q2, q3, q4) = EulerToQuaternion(pose[3], pose[4], pose[5]);
            rt.Q1 = q1;
            rt.Q2 = q2;
            rt.Q3 = q3;
            rt.Q4 = q4;
        }

        return rt;
    }

    /// <summary>
    /// Convert RPY (degrees) to quaternion [w, x, y, z]
    /// </summary>
    private static (double w, double x, double y, double z) EulerToQuaternion(
        double rollDeg, double pitchDeg, double yawDeg)
    {
        double roll = rollDeg * Math.PI / 180.0;
        double pitch = pitchDeg * Math.PI / 180.0;
        double yaw = yawDeg * Math.PI / 180.0;

        double cr = Math.Cos(roll / 2);
        double sr = Math.Sin(roll / 2);
        double cp = Math.Cos(pitch / 2);
        double sp = Math.Sin(pitch / 2);
        double cy = Math.Cos(yaw / 2);
        double sy = Math.Sin(yaw / 2);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;

        return (w, x, y, z);
    }
}

/// <summary>
/// Speed Data - Defines motion speeds.
/// Mirrors ABB 'speeddata' structure.
/// Usage: MoveL p1, v1000, z50, tool0;
/// </summary>
public class SpeedData
{
    public string Name { get; set; } = "";

    /// <summary>TCP speed (mm/s)</summary>
    public double VTcp { get; set; }

    /// <summary>Orientation speed (deg/s)</summary>
    public double VOrient { get; set; } = 500;

    /// <summary>Linear external axes speed (mm/s)</summary>
    public double VLeax { get; set; } = 5000;

    /// <summary>Rotational external axes speed (deg/s)</summary>
    public double VReax { get; set; } = 1000;

    /// <summary>
    /// Serialize to RPL inline format: [vTcp,vOrient,vLeax,vReax]
    /// </summary>
    public string ToRplString() => $"[{VTcp:F0},{VOrient:F0},{VLeax:F0},{VReax:F0}]";

    // Predefined speeds
    public static SpeedData V5 => new() { Name = "v5", VTcp = 5 };
    public static SpeedData V10 => new() { Name = "v10", VTcp = 10 };
    public static SpeedData V50 => new() { Name = "v50", VTcp = 50 };
    public static SpeedData V100 => new() { Name = "v100", VTcp = 100 };
    public static SpeedData V200 => new() { Name = "v200", VTcp = 200 };
    public static SpeedData V500 => new() { Name = "v500", VTcp = 500 };
    public static SpeedData V1000 => new() { Name = "v1000", VTcp = 1000 };
    public static SpeedData V2000 => new() { Name = "v2000", VTcp = 2000 };
    public static SpeedData VMax => new() { Name = "vmax", VTcp = 5000 };
}

/// <summary>
/// Zone Data - Defines how close TCP must come to a point before moving to next.
/// Mirrors ABB 'zonedata' structure.
/// Usage: MoveL p1, v1000, z50, tool0; (z50 = 50mm fly-by zone)
/// </summary>
public class ZoneData
{
    public string Name { get; set; } = "";

    /// <summary>True = stop point (fine), False = fly-by</summary>
    public bool FinePnt { get; set; }

    /// <summary>Path zone radius (mm) - how close TCP passes to the programmed point</summary>
    public double PZone { get; set; }

    /// <summary>Path zone for reorientation (mm)</summary>
    public double PZoneOri { get; set; }

    /// <summary>Path zone for external axes (mm)</summary>
    public double PZoneEax { get; set; }

    /// <summary>Zone orientation (deg)</summary>
    public double ZoneOri { get; set; }

    /// <summary>Zone for external rotational axes (deg)</summary>
    public double ZoneEax { get; set; }

    // Predefined zones
    public static ZoneData Fine => new() { Name = "fine", FinePnt = true, PZone = 0 };
    public static ZoneData Z1 => new() { Name = "z1", PZone = 1, PZoneOri = 1, ZoneOri = 0.3 };
    public static ZoneData Z5 => new() { Name = "z5", PZone = 5, PZoneOri = 8, ZoneOri = 0.8 };
    public static ZoneData Z10 => new() { Name = "z10", PZone = 10, PZoneOri = 15, ZoneOri = 1.5 };
    public static ZoneData Z50 => new() { Name = "z50", PZone = 50, PZoneOri = 75, ZoneOri = 8 };
    public static ZoneData Z100 => new() { Name = "z100", PZone = 100, PZoneOri = 150, ZoneOri = 15 };
    public static ZoneData Z200 => new() { Name = "z200", PZone = 200, PZoneOri = 300, ZoneOri = 30 };
}

/// <summary>
/// Weld Data - Steady-state welding parameters.
/// Mirrors ABB 'welddata' structure.
/// Controls power source during welding phase (after arc established, before crater fill).
/// </summary>
public class WeldData
{
    public string Id { get; init; } = Guid.NewGuid().ToString();
    public string Name { get; set; } = "";

    /// <summary>Weld speed / TCP speed during welding (mm/s)</summary>
    public double WeldSpeed { get; set; }

    /// <summary>Wire feed speed (m/min) - amperage proxy in CV mode</summary>
    public double WireFeedSpeed { get; set; }

    /// <summary>Reference voltage (V)</summary>
    public double Voltage { get; set; }

    /// <summary>Synergic line ID on power source</summary>
    public int SynergicLineId { get; set; }

    /// <summary>Arc regulation inductance</summary>
    public double Inductance { get; set; }

    public string Description { get; set; } = "";
}

/// <summary>
/// Seam Data - Transient parameters for Arc Start/End.
/// Mirrors ABB 'seamdata' structure.
/// Controls ignition (hot start) and crater fill phases.
/// </summary>
public class SeamData
{
    public string Id { get; init; } = Guid.NewGuid().ToString();
    public string Name { get; set; } = "";

    // Ignition Phase
    public double PreFlowTime { get; set; }
    public double PurgeTime { get; set; }
    public double IgnitionVoltage { get; set; }
    public double IgnitionWireFeed { get; set; }
    public double IgnitionMoveDelay { get; set; }

    // Crater/End Phase
    public double CraterFillTime { get; set; }
    public double CraterVoltage { get; set; }
    public double BurnbackTime { get; set; }
    public double PostFlowTime { get; set; }
}

/// <summary>
/// Weave Data - Geometric oscillation parameters.
/// Mirrors ABB 'weavedata' structure.
/// </summary>
public class WeaveData
{
    public string Name { get; set; } = "";

    /// <summary>Weave pattern shape</summary>
    public WeaveShape Shape { get; set; } = WeaveShape.Sine;

    /// <summary>Weave width / amplitude (mm)</summary>
    public double Width { get; set; }

    /// <summary>Weave length / pitch (mm)</summary>
    public double Length { get; set; }

    /// <summary>Dwell time at sides (s)</summary>
    public double DwellTime { get; set; }

    /// <summary>Bias from center (mm, positive = right)</summary>
    public double Bias { get; set; }
}

public enum WeaveShape
{
    Sine,
    Triangle,
    Trapezoid,
    Zigzag
}

/// <summary>
/// Motion type enumeration
/// </summary>
public enum MotionType
{
    MoveL,  // Linear motion
    MoveJ,  // Joint motion
    MoveC   // Circular motion
}

/// <summary>
/// Welding Job info for Intellisense.
/// Represents a synergic line/job on the power source.
/// </summary>
public class WeldingJobInfo
{
    public int Id { get; init; }
    public string Name { get; set; } = "";
    public string Material { get; set; } = "";
    public double WireFeedSpeed { get; set; }
    public double Voltage { get; set; }
    public string GasType { get; set; } = "";
    public double Thickness { get; set; }

    /// <summary>Default welding jobs for MIG/MAG</summary>
    public static List<WeldingJobInfo> DefaultJobs => new()
    {
        new() { Id = 1, Name = "Steel Thin", Material = "Steel", WireFeedSpeed = 4.5, Voltage = 18.5, GasType = "82% Ar / 18% CO2", Thickness = 1.0 },
        new() { Id = 2, Name = "Steel Medium", Material = "Steel", WireFeedSpeed = 7.0, Voltage = 22.0, GasType = "82% Ar / 18% CO2", Thickness = 2.0 },
        new() { Id = 3, Name = "Steel Thick", Material = "Steel", WireFeedSpeed = 10.0, Voltage = 26.0, GasType = "82% Ar / 18% CO2", Thickness = 3.0 },
        new() { Id = 4, Name = "Stainless", Material = "Stainless Steel", WireFeedSpeed = 6.0, Voltage = 20.0, GasType = "98% Ar / 2% CO2", Thickness = 1.5 },
        new() { Id = 5, Name = "Aluminum", Material = "Aluminum", WireFeedSpeed = 8.0, Voltage = 21.0, GasType = "100% Ar", Thickness = 2.0 },
        new() { Id = 10, Name = "Pulse Steel", Material = "Steel (Pulse)", WireFeedSpeed = 6.0, Voltage = 24.0, GasType = "82% Ar / 18% CO2", Thickness = 2.0 },
        new() { Id = 50, Name = "Pulse Aluminum", Material = "Aluminum (Pulse)", WireFeedSpeed = 9.0, Voltage = 22.0, GasType = "100% Ar", Thickness = 3.0 },
    };
}
