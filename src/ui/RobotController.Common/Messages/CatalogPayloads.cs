using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Summary of a robot model (for catalog listing)
/// </summary>
public class RobotModelSummary
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = string.Empty;

    [JsonPropertyName("name")]
    public string Name { get; set; } = string.Empty;

    [JsonPropertyName("manufacturer")]
    public string Manufacturer { get; set; } = string.Empty;

    [JsonPropertyName("dof")]
    public int Dof { get; set; } = 6;

    [JsonPropertyName("maxPayloadKg")]
    public double MaxPayloadKg { get; set; }

    [JsonPropertyName("reachMm")]
    public double ReachMm { get; set; }
}

/// <summary>
/// Response for GET_ROBOT_CATALOG request
/// </summary>
public class GetRobotCatalogResponse
{
    [JsonPropertyName("models")]
    public List<RobotModelSummary> Models { get; set; } = new();

    [JsonPropertyName("activeModelId")]
    public string ActiveModelId { get; set; } = string.Empty;

    [JsonPropertyName("activeInstanceId")]
    public string ActiveInstanceId { get; set; } = string.Empty;
}

/// <summary>
/// Request to select a robot model
/// </summary>
public class SelectRobotModelRequest
{
    [JsonPropertyName("modelId")]
    public string ModelId { get; set; } = string.Empty;
}

/// <summary>
/// Response for SELECT_ROBOT_MODEL request
/// </summary>
public class SelectRobotModelResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string? Error { get; set; }

    [JsonPropertyName("modelId")]
    public string ModelId { get; set; } = string.Empty;

    [JsonPropertyName("modelName")]
    public string ModelName { get; set; } = string.Empty;
}

/// <summary>
/// Response for GET_ACTIVE_ROBOT request
/// </summary>
public class GetActiveRobotResponse
{
    [JsonPropertyName("modelId")]
    public string ModelId { get; set; } = string.Empty;

    [JsonPropertyName("modelName")]
    public string ModelName { get; set; } = string.Empty;

    [JsonPropertyName("instanceId")]
    public string InstanceId { get; set; } = string.Empty;

    [JsonPropertyName("manufacturer")]
    public string Manufacturer { get; set; } = string.Empty;

    [JsonPropertyName("dof")]
    public int Dof { get; set; } = 6;

    [JsonPropertyName("maxPayloadKg")]
    public double MaxPayloadKg { get; set; }

    [JsonPropertyName("reachMm")]
    public double ReachMm { get; set; }
}

/// <summary>
/// Event published when robot config changes
/// </summary>
public class RobotConfigChangedEvent
{
    [JsonPropertyName("modelId")]
    public string ModelId { get; set; } = string.Empty;

    [JsonPropertyName("modelName")]
    public string ModelName { get; set; } = string.Empty;

    [JsonPropertyName("instanceId")]
    public string InstanceId { get; set; } = string.Empty;
}
