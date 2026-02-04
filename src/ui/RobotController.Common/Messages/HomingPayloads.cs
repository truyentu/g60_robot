using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Request to start homing
/// </summary>
public class StartHomingRequest
{
    [JsonPropertyName("jointIndex")]
    public int JointIndex { get; set; } = -1; // -1 = all joints

    [JsonPropertyName("method")]
    public string Method { get; set; } = "LIMIT_SWITCH";
}

/// <summary>
/// Response for start homing request
/// </summary>
public class StartHomingResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = string.Empty;
}

/// <summary>
/// Request to stop homing
/// </summary>
public class StopHomingRequest
{
    [JsonPropertyName("jointIndex")]
    public int JointIndex { get; set; } = -1; // -1 = all joints
}

/// <summary>
/// Response for stop homing request
/// </summary>
public class StopHomingResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = string.Empty;
}

/// <summary>
/// Status of a single joint's homing
/// </summary>
public class JointHomingStatus
{
    [JsonPropertyName("jointIndex")]
    public int JointIndex { get; set; }

    [JsonPropertyName("state")]
    public string State { get; set; } = "NOT_HOMED";

    [JsonPropertyName("progress")]
    public double Progress { get; set; }

    [JsonPropertyName("errorMessage")]
    public string ErrorMessage { get; set; } = string.Empty;

    [JsonPropertyName("limitSwitchActive")]
    public bool LimitSwitchActive { get; set; }

    [JsonPropertyName("currentPosition")]
    public double CurrentPosition { get; set; }

    // Convenience properties
    public bool IsHomed => State == "HOMED";
    public bool IsHoming => State == "HOMING_IN_PROGRESS";
    public bool HasError => State == "HOMING_ERROR";
}

/// <summary>
/// Response for get homing state request
/// </summary>
public class HomingStateResponse
{
    [JsonPropertyName("joints")]
    public List<JointHomingStatus> Joints { get; set; } = new();

    [JsonPropertyName("allHomed")]
    public bool AllHomed { get; set; }

    [JsonPropertyName("anyHoming")]
    public bool AnyHoming { get; set; }

    [JsonPropertyName("anyError")]
    public bool AnyError { get; set; }

    [JsonPropertyName("totalJoints")]
    public int TotalJoints { get; set; } = 6;

    [JsonPropertyName("homedCount")]
    public int HomedCount { get; set; }
}

/// <summary>
/// Event published when homing state changes
/// </summary>
public class HomingStateChangedEvent
{
    [JsonPropertyName("jointIndex")]
    public int JointIndex { get; set; }

    [JsonPropertyName("previousState")]
    public string PreviousState { get; set; } = string.Empty;

    [JsonPropertyName("newState")]
    public string NewState { get; set; } = string.Empty;

    [JsonPropertyName("errorMessage")]
    public string ErrorMessage { get; set; } = string.Empty;
}
