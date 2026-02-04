using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// SET_OVERRIDE request
/// </summary>
public class SetOverrideRequest
{
    [JsonPropertyName("programOverride")]
    public int ProgramOverride { get; set; } = -1;  // -1 = don't change

    [JsonPropertyName("jogOverride")]
    public int JogOverride { get; set; } = -1;

    [JsonPropertyName("manualOverride")]
    public int ManualOverride { get; set; } = -1;
}

/// <summary>
/// SET_OVERRIDE response
/// </summary>
public class SetOverrideResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";

    [JsonPropertyName("programOverride")]
    public int ProgramOverride { get; set; } = 100;

    [JsonPropertyName("jogOverride")]
    public int JogOverride { get; set; } = 100;

    [JsonPropertyName("manualOverride")]
    public int ManualOverride { get; set; } = 100;
}

/// <summary>
/// GET_OVERRIDE response
/// </summary>
public class GetOverrideResponse
{
    [JsonPropertyName("programOverride")]
    public int ProgramOverride { get; set; } = 100;

    [JsonPropertyName("jogOverride")]
    public int JogOverride { get; set; } = 100;

    [JsonPropertyName("manualOverride")]
    public int ManualOverride { get; set; } = 100;
}

/// <summary>
/// OVERRIDE_CHANGED event
/// </summary>
public class OverrideChangedEvent
{
    [JsonPropertyName("programOverride")]
    public int ProgramOverride { get; set; } = 100;

    [JsonPropertyName("jogOverride")]
    public int JogOverride { get; set; } = 100;

    [JsonPropertyName("manualOverride")]
    public int ManualOverride { get; set; } = 100;

    [JsonPropertyName("changedBy")]
    public string ChangedBy { get; set; } = "user";
}
