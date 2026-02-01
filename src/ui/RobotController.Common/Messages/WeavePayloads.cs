using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

// ============================================================================
// Enums
// ============================================================================

public enum WeavePatternType
{
    None = 0,
    Linear = 1,
    Triangular = 2,
    Sinusoidal = 3,
    Circular = 4,
    Figure8 = 5,
    Crescent = 6,
    Custom = 7
}

// ============================================================================
// Weave Parameters
// ============================================================================

public record WeaveParamsData
{
    [JsonPropertyName("patternType")]
    public int PatternType { get; init; } = (int)WeavePatternType.Sinusoidal;

    [JsonPropertyName("amplitude")]
    public double Amplitude { get; init; } = 3.0;

    [JsonPropertyName("wavelength")]
    public double Wavelength { get; init; } = 10.0;

    [JsonPropertyName("frequency")]
    public double Frequency { get; init; } = 1.0;

    [JsonPropertyName("dwellLeft")]
    public double DwellLeft { get; init; }

    [JsonPropertyName("dwellRight")]
    public double DwellRight { get; init; }

    [JsonPropertyName("dwellCenter")]
    public double DwellCenter { get; init; }

    [JsonPropertyName("phaseOffset")]
    public double PhaseOffset { get; init; }

    [JsonPropertyName("speedAtEdge")]
    public double SpeedAtEdge { get; init; } = 0.8;

    [JsonPropertyName("speedAtCenter")]
    public double SpeedAtCenter { get; init; } = 1.0;

    [JsonPropertyName("useFrequency")]
    public bool UseFrequency { get; init; }
}

// ============================================================================
// Weave Control
// ============================================================================

public record WeaveControlRequest
{
    [JsonPropertyName("action")]
    public string Action { get; init; } = "";

    [JsonPropertyName("params")]
    public WeaveParamsData? Params { get; init; }

    [JsonPropertyName("adjustParam")]
    public string AdjustParam { get; init; } = "";

    [JsonPropertyName("adjustValue")]
    public double AdjustValue { get; init; }
}

public record WeaveControlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Weave Status
// ============================================================================

public record WeaveStatusResponse
{
    [JsonPropertyName("enabled")]
    public bool Enabled { get; init; }

    [JsonPropertyName("active")]
    public bool Active { get; init; }

    [JsonPropertyName("patternType")]
    public int PatternType { get; init; }

    [JsonPropertyName("currentPhase")]
    public double CurrentPhase { get; init; }

    [JsonPropertyName("currentAmplitude")]
    public double CurrentAmplitude { get; init; }

    [JsonPropertyName("lateralOffset")]
    public double LateralOffset { get; init; }

    [JsonPropertyName("cycleCount")]
    public int CycleCount { get; init; }

    [JsonPropertyName("totalDistance")]
    public double TotalDistance { get; init; }
}

// ============================================================================
// Weave Preview
// ============================================================================

public record WeavePreviewRequest
{
    [JsonPropertyName("params")]
    public WeaveParamsData? Params { get; init; }

    [JsonPropertyName("numCycles")]
    public int NumCycles { get; init; } = 3;

    [JsonPropertyName("pointsPerCycle")]
    public int PointsPerCycle { get; init; } = 64;
}

public record WeavePreviewResponse
{
    [JsonPropertyName("xPoints")]
    public double[] XPoints { get; init; } = Array.Empty<double>();

    [JsonPropertyName("yPoints")]
    public double[] YPoints { get; init; } = Array.Empty<double>();

    [JsonPropertyName("pathLengthRatio")]
    public double PathLengthRatio { get; init; }
}
