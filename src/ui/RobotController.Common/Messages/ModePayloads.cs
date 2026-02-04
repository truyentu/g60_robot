using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Response for GET_OPERATION_MODE
/// </summary>
public class GetOperationModeResponse
{
    [JsonPropertyName("mode")]
    public string Mode { get; set; } = "MANUAL";

    [JsonPropertyName("maxLinearVelocity")]
    public double MaxLinearVelocity { get; set; }

    [JsonPropertyName("maxJointVelocity")]
    public double MaxJointVelocity { get; set; }

    [JsonPropertyName("deadmanRequired")]
    public bool DeadmanRequired { get; set; }

    [JsonPropertyName("safetyFenceRequired")]
    public bool SafetyFenceRequired { get; set; }

    [JsonPropertyName("externalControlAllowed")]
    public bool ExternalControlAllowed { get; set; }
}

/// <summary>
/// Request for SET_OPERATION_MODE
/// </summary>
public class SetOperationModeRequest
{
    [JsonPropertyName("mode")]
    public string Mode { get; set; } = "MANUAL";
}

/// <summary>
/// Response for SET_OPERATION_MODE
/// </summary>
public class SetOperationModeResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("newMode")]
    public string NewMode { get; set; } = "";

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";

    [JsonPropertyName("missingRequirements")]
    public List<string> MissingRequirements { get; set; } = new();
}

/// <summary>
/// Request for GET_MODE_REQUIREMENTS
/// </summary>
public class GetModeRequirementsRequest
{
    [JsonPropertyName("targetMode")]
    public string TargetMode { get; set; } = "MANUAL";
}

/// <summary>
/// Response for GET_MODE_REQUIREMENTS
/// </summary>
public class GetModeRequirementsResponse
{
    [JsonPropertyName("targetMode")]
    public string TargetMode { get; set; } = "";

    [JsonPropertyName("robotStopped")]
    public bool RobotStopped { get; set; }

    [JsonPropertyName("safetyFenceClosed")]
    public bool SafetyFenceClosed { get; set; }

    [JsonPropertyName("deadmanReleased")]
    public bool DeadmanReleased { get; set; }

    [JsonPropertyName("noActiveAlarms")]
    public bool NoActiveAlarms { get; set; }

    [JsonPropertyName("homingComplete")]
    public bool HomingComplete { get; set; }

    [JsonPropertyName("missingItems")]
    public List<string> MissingItems { get; set; } = new();

    [JsonPropertyName("canTransition")]
    public bool CanTransition { get; set; }
}

/// <summary>
/// Event for OPERATION_MODE_CHANGED
/// </summary>
public class OperationModeChangedEvent
{
    [JsonPropertyName("previousMode")]
    public string PreviousMode { get; set; } = "";

    [JsonPropertyName("newMode")]
    public string NewMode { get; set; } = "";

    [JsonPropertyName("maxLinearVelocity")]
    public double MaxLinearVelocity { get; set; }

    [JsonPropertyName("maxJointVelocity")]
    public double MaxJointVelocity { get; set; }
}
