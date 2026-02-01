using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

// ============================================================================
// Enums
// ============================================================================

public enum WeldingState
{
    Idle = 0,
    PreFlow = 1,
    ArcStart = 2,
    Welding = 3,
    CraterFill = 4,
    ArcEnd = 5,
    BurnBack = 6,
    PostFlow = 7,
    Fault = 8,
    EmergencyStop = 9
}

public enum WeldingFault
{
    None = 0,
    NoGas = 1,
    NoWire = 2,
    ArcFail = 3,
    ArcLost = 4,
    OverCurrent = 5,
    OverVoltage = 6,
    StickWire = 7,
    PowerSupplyFault = 8,
    CommunicationFault = 9,
    SafetyFault = 10,
    ThermalFault = 11
}

public enum WeldingProcess
{
    MigMag = 0,
    Tig = 1,
    Spot = 2,
    Plasma = 3,
    Laser = 4
}

public enum TransferMode
{
    ShortArc = 0,
    Globular = 1,
    Spray = 2,
    Pulse = 3,
    Cmt = 4,
    Stt = 5
}

// ============================================================================
// Welding Job
// ============================================================================

public record WeldingJobData
{
    [JsonPropertyName("name")]
    public string Name { get; init; } = "Default";

    [JsonPropertyName("process")]
    public int Process { get; init; }

    [JsonPropertyName("transferMode")]
    public int TransferMode { get; init; }

    [JsonPropertyName("current")]
    public float Current { get; init; } = 180;

    [JsonPropertyName("voltage")]
    public float Voltage { get; init; } = 22;

    [JsonPropertyName("wireSpeed")]
    public float WireSpeed { get; init; } = 8;

    [JsonPropertyName("travelSpeed")]
    public float TravelSpeed { get; init; } = 10;

    [JsonPropertyName("preFlowTime")]
    public uint PreFlowTime { get; init; } = 300;

    [JsonPropertyName("postFlowTime")]
    public uint PostFlowTime { get; init; } = 500;

    [JsonPropertyName("craterFillTime")]
    public uint CraterFillTime { get; init; } = 500;

    [JsonPropertyName("synergicMode")]
    public bool SynergicMode { get; init; } = true;

    [JsonPropertyName("synergicProgram")]
    public int SynergicProgram { get; init; } = 1;
}

// ============================================================================
// Welding Control
// ============================================================================

public record WeldingControlRequest
{
    [JsonPropertyName("action")]
    public string Action { get; init; } = "";

    [JsonPropertyName("job")]
    public WeldingJobData? Job { get; init; }
}

public record WeldingControlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("state")]
    public string State { get; init; } = "";

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Welding Status
// ============================================================================

public record WeldingStatusResponse
{
    [JsonPropertyName("state")]
    public string State { get; init; } = "Idle";

    [JsonPropertyName("fault")]
    public string Fault { get; init; } = "None";

    [JsonPropertyName("isWelding")]
    public bool IsWelding { get; init; }

    [JsonPropertyName("hasFault")]
    public bool HasFault { get; init; }

    [JsonPropertyName("isReady")]
    public bool IsReady { get; init; }

    [JsonPropertyName("actualCurrent")]
    public float ActualCurrent { get; init; }

    [JsonPropertyName("actualVoltage")]
    public float ActualVoltage { get; init; }

    [JsonPropertyName("actualWireSpeed")]
    public float ActualWireSpeed { get; init; }

    [JsonPropertyName("arcPresent")]
    public bool ArcPresent { get; init; }

    [JsonPropertyName("gasFlowOk")]
    public bool GasFlowOk { get; init; }

    [JsonPropertyName("weldTime")]
    public uint WeldTime { get; init; }

    [JsonPropertyName("weldDistance")]
    public float WeldDistance { get; init; }
}

// ============================================================================
// Parameter Adjustment
// ============================================================================

public record WeldingAdjustRequest
{
    [JsonPropertyName("parameter")]
    public string Parameter { get; init; } = "";

    [JsonPropertyName("value")]
    public float Value { get; init; }
}

public record WeldingAdjustResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("newValue")]
    public float NewValue { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}
