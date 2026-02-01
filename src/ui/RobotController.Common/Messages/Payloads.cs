using System.Text.Json;
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// PONG response payload
/// </summary>
public class PongPayload
{
    [JsonPropertyName("core_version")]
    public string CoreVersion { get; set; } = string.Empty;

    [JsonPropertyName("uptime_ms")]
    public long UptimeMs { get; set; }
}

/// <summary>
/// STATUS payload (published periodically)
/// </summary>
public class StatusPayload
{
    [JsonPropertyName("state")]
    public string State { get; set; } = "IDLE";

    [JsonPropertyName("mode")]
    public string Mode { get; set; } = "MANUAL";

    [JsonPropertyName("joints")]
    public List<double> Joints { get; set; } = new();

    [JsonPropertyName("tcp_position")]
    public List<double> TcpPosition { get; set; } = new();

    [JsonPropertyName("errors")]
    public List<string> Errors { get; set; } = new();

    [JsonPropertyName("homed")]
    public bool Homed { get; set; }

    [JsonPropertyName("enabled")]
    public bool Enabled { get; set; }
}

/// <summary>
/// Joint positions payload
/// </summary>
public class JointPositionsPayload
{
    [JsonPropertyName("joints")]
    public double[] Joints { get; set; } = new double[6];

    [JsonPropertyName("unit")]
    public string Unit { get; set; } = "degrees";
}

/// <summary>
/// Error payload
/// </summary>
public class ErrorPayload
{
    [JsonPropertyName("code")]
    public int Code { get; set; }

    [JsonPropertyName("message")]
    public string Message { get; set; } = string.Empty;

    [JsonPropertyName("details")]
    public string? Details { get; set; }
}

/// <summary>
/// Command payload
/// </summary>
public class CommandPayload
{
    [JsonPropertyName("command")]
    public string Command { get; set; } = string.Empty;

    [JsonPropertyName("parameters")]
    public JsonElement Parameters { get; set; }
}
