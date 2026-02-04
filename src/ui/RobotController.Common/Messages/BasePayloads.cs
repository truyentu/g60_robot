using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Frame position and orientation data
/// </summary>
public class FrameData
{
    [JsonPropertyName("x")]
    public double X { get; set; }

    [JsonPropertyName("y")]
    public double Y { get; set; }

    [JsonPropertyName("z")]
    public double Z { get; set; }

    [JsonPropertyName("rx")]
    public double Rx { get; set; }

    [JsonPropertyName("ry")]
    public double Ry { get; set; }

    [JsonPropertyName("rz")]
    public double Rz { get; set; }
}

/// <summary>
/// Base frame data
/// </summary>
public class BaseFrameData
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";

    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("description")]
    public string Description { get; set; } = "";

    [JsonPropertyName("frame")]
    public FrameData Frame { get; set; } = new();

    [JsonPropertyName("isActive")]
    public bool IsActive { get; set; }
}

// ============================================================================
// Request/Response Classes
// ============================================================================

/// <summary>
/// Response for GET_BASE_LIST
/// </summary>
public class GetBaseListResponse
{
    [JsonPropertyName("bases")]
    public List<BaseFrameData> Bases { get; set; } = new();

    [JsonPropertyName("activeBaseId")]
    public string ActiveBaseId { get; set; } = "";
}

/// <summary>
/// Request for GET_BASE
/// </summary>
public class GetBaseRequest
{
    [JsonPropertyName("baseId")]
    public string BaseId { get; set; } = "";
}

/// <summary>
/// Response for GET_BASE
/// </summary>
public class GetBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("base")]
    public BaseFrameData Base { get; set; } = new();

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Request for CREATE_BASE
/// </summary>
public class CreateBaseRequest
{
    [JsonPropertyName("base")]
    public BaseFrameData Base { get; set; } = new();
}

/// <summary>
/// Response for CREATE_BASE
/// </summary>
public class CreateBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Request for UPDATE_BASE
/// </summary>
public class UpdateBaseRequest
{
    [JsonPropertyName("baseId")]
    public string BaseId { get; set; } = "";

    [JsonPropertyName("base")]
    public BaseFrameData Base { get; set; } = new();
}

/// <summary>
/// Response for UPDATE_BASE
/// </summary>
public class UpdateBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Request for DELETE_BASE
/// </summary>
public class DeleteBaseRequest
{
    [JsonPropertyName("baseId")]
    public string BaseId { get; set; } = "";
}

/// <summary>
/// Response for DELETE_BASE
/// </summary>
public class DeleteBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Request for SELECT_BASE
/// </summary>
public class SelectBaseRequest
{
    [JsonPropertyName("baseId")]
    public string BaseId { get; set; } = "";
}

/// <summary>
/// Response for SELECT_BASE
/// </summary>
public class SelectBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Response for GET_ACTIVE_BASE
/// </summary>
public class GetActiveBaseResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("base")]
    public BaseFrameData Base { get; set; } = new();
}

// ============================================================================
// Calibration Classes
// ============================================================================

/// <summary>
/// Request for START_BASE_CALIBRATION
/// </summary>
public class StartBaseCalibrationRequest
{
    [JsonPropertyName("method")]
    public string Method { get; set; } = "THREE_POINT";
}

/// <summary>
/// Response for START_BASE_CALIBRATION
/// </summary>
public class StartBaseCalibrationResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("pointsRequired")]
    public int PointsRequired { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Request for RECORD_BASE_POINT
/// </summary>
public class RecordBasePointRequest
{
    [JsonPropertyName("pointIndex")]
    public int PointIndex { get; set; }

    [JsonPropertyName("jointAngles")]
    public List<double> JointAngles { get; set; } = new();

    [JsonPropertyName("tcpPosition")]
    public List<double> TcpPosition { get; set; } = new();
}

/// <summary>
/// Response for RECORD_BASE_POINT
/// </summary>
public class RecordBasePointResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("pointIndex")]
    public int PointIndex { get; set; }

    [JsonPropertyName("pointName")]
    public string PointName { get; set; } = "";

    [JsonPropertyName("totalPoints")]
    public int TotalPoints { get; set; }

    [JsonPropertyName("recordedPoints")]
    public int RecordedPoints { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Response for FINISH_BASE_CALIBRATION
/// </summary>
public class FinishBaseCalibrationResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("calculatedFrame")]
    public FrameData CalculatedFrame { get; set; } = new();

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

/// <summary>
/// Response for GET_BASE_CALIBRATION_STATUS
/// </summary>
public class BaseCalibrationStatusResponse
{
    [JsonPropertyName("state")]
    public string State { get; set; } = "IDLE";

    [JsonPropertyName("method")]
    public string Method { get; set; } = "THREE_POINT";

    [JsonPropertyName("pointsRequired")]
    public int PointsRequired { get; set; }

    [JsonPropertyName("pointsRecorded")]
    public int PointsRecorded { get; set; }

    [JsonPropertyName("currentPointName")]
    public string CurrentPointName { get; set; } = "";

    [JsonPropertyName("errorMessage")]
    public string ErrorMessage { get; set; } = "";
}

/// <summary>
/// Event for BASE_CHANGED
/// </summary>
public class BaseChangedEvent
{
    [JsonPropertyName("baseId")]
    public string BaseId { get; set; } = "";

    [JsonPropertyName("baseName")]
    public string BaseName { get; set; } = "";
}
