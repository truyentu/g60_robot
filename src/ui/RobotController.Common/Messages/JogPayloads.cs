using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

public enum JogMode
{
    Joint = 0,
    Cartesian = 1
}

public enum JogType
{
    Continuous = 0,
    Incremental = 1
}

public enum JogFrame
{
    World = 0,
    Base = 1,
    Tool = 2,
    User1 = 3,
    User2 = 4
}

public class JogStartRequest
{
    [JsonPropertyName("enableDeadman")]
    public bool EnableDeadman { get; set; } = true;
}

public class JogStartResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

public class JogMoveRequest
{
    [JsonPropertyName("mode")]
    public int Mode { get; set; }

    [JsonPropertyName("axis")]
    public int Axis { get; set; }

    [JsonPropertyName("direction")]
    public int Direction { get; set; }

    [JsonPropertyName("speedPercent")]
    public double SpeedPercent { get; set; }

    [JsonPropertyName("frame")]
    public int Frame { get; set; }
}

public class JogStepRequest
{
    [JsonPropertyName("mode")]
    public int Mode { get; set; }

    [JsonPropertyName("axis")]
    public int Axis { get; set; }

    [JsonPropertyName("direction")]
    public int Direction { get; set; }

    [JsonPropertyName("increment")]
    public double Increment { get; set; }

    [JsonPropertyName("speedPercent")]
    public double SpeedPercent { get; set; }

    [JsonPropertyName("frame")]
    public int Frame { get; set; }
}

public class JogMoveResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

public class JogStatePayload
{
    [JsonPropertyName("enabled")]
    public bool Enabled { get; set; }

    [JsonPropertyName("is_moving")]
    public bool IsMoving { get; set; }

    [JsonPropertyName("current_mode")]
    public int CurrentMode { get; set; }

    [JsonPropertyName("current_axis")]
    public int CurrentAxis { get; set; }

    [JsonPropertyName("current_direction")]
    public int CurrentDirection { get; set; }

    [JsonPropertyName("current_speed")]
    public double CurrentSpeed { get; set; }
}
