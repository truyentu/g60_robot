namespace RobotController.Common.Messages;

// ============================================================================
// Tool Data Models
// ============================================================================

/// <summary>
/// TCP offset from flange
/// </summary>
public class ToolTCPData
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }
}

/// <summary>
/// Tool inertia data
/// </summary>
public class ToolInertiaData
{
    public double Mass { get; set; }
    public double CogX { get; set; }
    public double CogY { get; set; }
    public double CogZ { get; set; }
}

/// <summary>
/// Complete tool data
/// </summary>
public class ToolData
{
    public string Id { get; set; } = string.Empty;
    public string Name { get; set; } = string.Empty;
    public string Description { get; set; } = string.Empty;
    public ToolTCPData Tcp { get; set; } = new();
    public ToolInertiaData Inertia { get; set; } = new();
    public bool IsActive { get; set; }
    public string VisualMeshPath { get; set; } = string.Empty;
    public double[] MeshOffset { get; set; } = new double[6];
    public double MeshScale { get; set; } = 1.0;  // Scale factor for STL (1.0=mm, 1000.0=meters→mm)
}

// ============================================================================
// Request/Response Payloads
// ============================================================================

public class GetToolListResponse
{
    public List<ToolData> Tools { get; set; } = new();
    public string ActiveToolId { get; set; } = string.Empty;
}

public class GetToolRequest
{
    public string ToolId { get; set; } = string.Empty;
}

public class GetToolResponse
{
    public bool Success { get; set; }
    public ToolData? Tool { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class CreateToolRequest
{
    public ToolData Tool { get; set; } = new();
}

public class CreateToolResponse
{
    public bool Success { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class UpdateToolRequest
{
    public string ToolId { get; set; } = string.Empty;
    public ToolData Tool { get; set; } = new();
}

public class UpdateToolResponse
{
    public bool Success { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class DeleteToolRequest
{
    public string ToolId { get; set; } = string.Empty;
}

public class DeleteToolResponse
{
    public bool Success { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class SelectToolRequest
{
    public string ToolId { get; set; } = string.Empty;
}

public class SelectToolResponse
{
    public bool Success { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class GetActiveToolResponse
{
    public bool Success { get; set; }
    public ToolData? Tool { get; set; }
    public string Error { get; set; } = string.Empty;
}

// ============================================================================
// Calibration Payloads
// ============================================================================

public class StartCalibrationRequest
{
    public string Method { get; set; } = "FOUR_POINT";
}

public class StartCalibrationResponse
{
    public bool Success { get; set; }
    public int PointsRequired { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class RecordCalibrationPointRequest
{
    public List<double> JointAngles { get; set; } = new();
}

public class RecordCalibrationPointResponse
{
    public bool Success { get; set; }
    public int PointIndex { get; set; }
    public int TotalRequired { get; set; }
    public bool IsComplete { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class FinishCalibrationResponse
{
    public bool Success { get; set; }
    public ToolTCPData? CalculatedTcp { get; set; }
    public double ResidualError { get; set; }
    public string Error { get; set; } = string.Empty;
}

public class CalibrationStatusResponse
{
    public string State { get; set; } = "IDLE";
    public string Method { get; set; } = "FOUR_POINT";
    public int PointsRecorded { get; set; }
    public int PointsRequired { get; set; }
    public string Error { get; set; } = string.Empty;
}

// ============================================================================
// Event Payloads
// ============================================================================

public class ToolChangedEvent
{
    public string ToolId { get; set; } = string.Empty;
    public string ToolName { get; set; } = string.Empty;
}
