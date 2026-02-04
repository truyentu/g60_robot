using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

/// <summary>
/// Robot Package info payload (lightweight)
/// </summary>
public class RobotPackageInfoPayload
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";

    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("manufacturer")]
    public string Manufacturer { get; set; } = "";

    [JsonPropertyName("model_type")]
    public string ModelType { get; set; } = "";

    [JsonPropertyName("payload_kg")]
    public double PayloadKg { get; set; }

    [JsonPropertyName("reach_mm")]
    public double ReachMm { get; set; }

    [JsonPropertyName("dof")]
    public int Dof { get; set; } = 6;

    [JsonPropertyName("has_meshes")]
    public bool HasMeshes { get; set; }

    [JsonPropertyName("thumbnail_path")]
    public string ThumbnailPath { get; set; } = "";
}

/// <summary>
/// Joint mesh info
/// </summary>
public class JointMeshInfoPayload
{
    [JsonPropertyName("visual_mesh")]
    public string VisualMesh { get; set; } = "";

    [JsonPropertyName("collision_mesh")]
    public string CollisionMesh { get; set; } = "";

    [JsonPropertyName("origin")]
    public double[] Origin { get; set; } = new double[3];
}

/// <summary>
/// Joint definition payload
/// </summary>
public class JointDefinitionPayload
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("type")]
    public string Type { get; set; } = "revolute";

    [JsonPropertyName("dh_a")]
    public double DhA { get; set; }

    [JsonPropertyName("dh_alpha")]
    public double DhAlpha { get; set; }

    [JsonPropertyName("dh_d")]
    public double DhD { get; set; }

    [JsonPropertyName("dh_theta_offset")]
    public double DhThetaOffset { get; set; }

    [JsonPropertyName("limit_min")]
    public double LimitMin { get; set; }

    [JsonPropertyName("limit_max")]
    public double LimitMax { get; set; }

    [JsonPropertyName("velocity_max")]
    public double VelocityMax { get; set; }

    [JsonPropertyName("acceleration_max")]
    public double AccelerationMax { get; set; }

    [JsonPropertyName("mesh")]
    public JointMeshInfoPayload? Mesh { get; set; }
}

/// <summary>
/// Full robot package payload
/// </summary>
public class RobotPackagePayload
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("id")]
    public string Id { get; set; } = "";

    [JsonPropertyName("manufacturer")]
    public string Manufacturer { get; set; } = "";

    [JsonPropertyName("model_type")]
    public string ModelType { get; set; } = "";

    [JsonPropertyName("payload_kg")]
    public double PayloadKg { get; set; }

    [JsonPropertyName("reach_mm")]
    public double ReachMm { get; set; }

    [JsonPropertyName("dh_convention")]
    public string DhConvention { get; set; } = "modified_dh";

    [JsonPropertyName("joints")]
    public List<JointDefinitionPayload> Joints { get; set; } = new();

    [JsonPropertyName("base_mesh")]
    public string BaseMesh { get; set; } = "";

    [JsonPropertyName("base_origin")]
    public double[] BaseOrigin { get; set; } = new double[6];

    [JsonPropertyName("flange_offset")]
    public double[] FlangeOffset { get; set; } = new double[3];

    [JsonPropertyName("home_position")]
    public double[] HomePosition { get; set; } = Array.Empty<double>();

    [JsonPropertyName("package_path")]
    public string PackagePath { get; set; } = "";
}

/// <summary>
/// Response for GET_ROBOT_PACKAGES
/// </summary>
public class GetRobotPackagesResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("packages")]
    public List<RobotPackageInfoPayload> Packages { get; set; } = new();
}

/// <summary>
/// Request for LOAD_ROBOT_PACKAGE
/// </summary>
public class LoadRobotPackageRequest
{
    [JsonPropertyName("package_id")]
    public string PackageId { get; set; } = "";
}

/// <summary>
/// Response for LOAD_ROBOT_PACKAGE
/// </summary>
public class LoadRobotPackageResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string? Error { get; set; }

    [JsonPropertyName("package")]
    public RobotPackagePayload? Package { get; set; }
}

// ============================================================================
// Program Execution Payloads
// ============================================================================

/// <summary>
/// Response for LOAD_PROGRAM
/// </summary>
public class LoadProgramResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string? Error { get; set; }

    [JsonPropertyName("program_name")]
    public string ProgramName { get; set; } = "";
}

/// <summary>
/// Response for STEP_PROGRAM
/// </summary>
public class StepProgramResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("state")]
    public string State { get; set; } = "IDLE";

    [JsonPropertyName("current_line")]
    public int CurrentLine { get; set; }
}

/// <summary>
/// Response for GET_PROGRAM_STATE
/// </summary>
public class GetProgramStateResponse
{
    [JsonPropertyName("state")]
    public string State { get; set; } = "IDLE";

    [JsonPropertyName("current_line")]
    public int CurrentLine { get; set; }

    [JsonPropertyName("program_name")]
    public string ProgramName { get; set; } = "";
}

/// <summary>
/// Response for GET_POINTS
/// </summary>
public class GetPointsResponse
{
    [JsonPropertyName("points")]
    public Dictionary<string, double[]> Points { get; set; } = new();
}
