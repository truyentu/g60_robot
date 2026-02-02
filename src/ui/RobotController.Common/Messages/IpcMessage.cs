using System.Text.Json;
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Base class for all IPC messages
/// </summary>
public class IpcMessage
{
    [JsonPropertyName("type")]
    public string Type { get; set; } = string.Empty;

    [JsonPropertyName("id")]
    public string Id { get; set; } = string.Empty;

    [JsonPropertyName("timestamp")]
    public long Timestamp { get; set; }

    [JsonPropertyName("payload")]
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
    public JsonElement Payload { get; set; }

    /// <summary>
    /// Create a new message with auto-generated ID and timestamp
    /// </summary>
    public static T Create<T>(string type) where T : IpcMessage, new()
    {
        return new T
        {
            Type = type,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };
    }

    /// <summary>
    /// Create a new message with auto-generated ID and timestamp
    /// </summary>
    public static IpcMessage Create(string type, JsonElement payload = default)
    {
        return new IpcMessage
        {
            Type = type,
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            Payload = payload
        };
    }

    /// <summary>
    /// Serialize message to JSON string
    /// </summary>
    public string Serialize()
    {
        return JsonSerializer.Serialize(this, new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            WriteIndented = false
        });
    }

    /// <summary>
    /// Deserialize from JSON string
    /// </summary>
    public static T? Deserialize<T>(string json) where T : IpcMessage
    {
        try
        {
            return JsonSerializer.Deserialize<T>(json, new JsonSerializerOptions
            {
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase
            });
        }
        catch
        {
            return null;
        }
    }

    /// <summary>
    /// Deserialize from JSON string to base IpcMessage
    /// </summary>
    public static IpcMessage? Deserialize(string json)
    {
        return Deserialize<IpcMessage>(json);
    }
}
