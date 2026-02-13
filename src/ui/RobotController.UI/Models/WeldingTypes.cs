namespace RobotController.UI.Models;

/// <summary>
/// Robot Target - Position + Orientation (Euler ABC).
/// KRL E6POS format: {X val, Y val, Z val, A val, B val, C val}
/// </summary>
public class RobTarget
{
    public string Name { get; set; } = "";

    // Position (mm)
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    // Orientation (Euler angles A, B, C in degrees - KRL convention)
    public double A { get; set; }
    public double B { get; set; }
    public double C { get; set; }

    /// <summary>
    /// Serialize to KRL E6POS aggregate format: {X val, Y val, Z val, A val, B val, C val}
    /// </summary>
    public string ToKrlString()
    {
        return $"{{X {X:F2}, Y {Y:F2}, Z {Z:F2}, A {A:F2}, B {B:F2}, C {C:F2}}}";
    }

    /// <summary>
    /// Create from current robot TCP pose [x,y,z,a,b,c]
    /// Euler angles A, B, C in degrees (KRL convention).
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
            rt.A = pose[3];
            rt.B = pose[4];
            rt.C = pose[5];
        }

        return rt;
    }
}

/// <summary>
/// KRL Approximation type for motion commands.
/// In KUKA KRL: exact stop = no suffix; fly-by = C_DIS, C_PTP, C_VEL, or C_ORI.
/// Velocity is set via $VEL.CP (m/s) or $VEL_AXIS[] (%) system variables.
/// Tool/Base via $TOOL = TOOL_DATA[N], $BASE = BASE_DATA[N].
/// </summary>
public enum ApproximationType
{
    EXACT,   // No suffix - exact stop point
    C_PTP,   // PTP approximation ($APO.CPTP in %)
    C_DIS,   // Distance approximation ($APO.CDIS in mm)
    C_VEL,   // Velocity approximation ($APO.CVEL in %)
    C_ORI    // Orientation approximation ($APO.CORI in deg)
}

/// <summary>
/// Weld Data - Steady-state welding parameters.
/// KRL welding configuration.
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
/// KRL seam configuration.
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
/// KRL weave configuration.
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
/// Motion type enumeration (KRL)
/// </summary>
public enum MotionType
{
    PTP,    // Point-to-Point (joint interpolation)
    LIN,    // Linear motion
    CIRC    // Circular motion
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
