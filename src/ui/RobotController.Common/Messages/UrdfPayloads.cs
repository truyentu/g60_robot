using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

// ============================================================================
// URDF Import Payloads
// ============================================================================

/// <summary>
/// Parsed URDF joint data
/// </summary>
public class UrdfJointPayload
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("type")]
    public string Type { get; set; } = "revolute";

    [JsonPropertyName("parent_link")]
    public string ParentLink { get; set; } = "";

    [JsonPropertyName("child_link")]
    public string ChildLink { get; set; } = "";

    [JsonPropertyName("origin_xyz")]
    public double[] OriginXyz { get; set; } = new double[3];

    [JsonPropertyName("origin_rpy")]
    public double[] OriginRpy { get; set; } = new double[3];

    [JsonPropertyName("axis")]
    public double[] Axis { get; set; } = new double[3];

    [JsonPropertyName("limit_lower_deg")]
    public double LimitLowerDeg { get; set; }

    [JsonPropertyName("limit_upper_deg")]
    public double LimitUpperDeg { get; set; }

    [JsonPropertyName("limit_velocity_deg")]
    public double LimitVelocityDeg { get; set; }
}

/// <summary>
/// Parsed URDF link data
/// </summary>
public class UrdfLinkPayload
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("visual_mesh")]
    public string VisualMesh { get; set; } = "";

    [JsonPropertyName("collision_mesh")]
    public string CollisionMesh { get; set; } = "";
}

/// <summary>
/// Parsed URDF model
/// </summary>
public class UrdfModelPayload
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("base_link")]
    public string BaseLink { get; set; } = "";

    [JsonPropertyName("joint_order")]
    public List<string> JointOrder { get; set; } = new();

    [JsonPropertyName("links")]
    public List<UrdfLinkPayload> Links { get; set; } = new();

    [JsonPropertyName("joints")]
    public List<UrdfJointPayload> Joints { get; set; } = new();
}

/// <summary>
/// Request for PARSE_URDF
/// </summary>
public class ParseUrdfRequest
{
    [JsonPropertyName("urdf_content")]
    public string UrdfContent { get; set; } = "";

    [JsonPropertyName("is_file_path")]
    public bool IsFilePath { get; set; }
}

/// <summary>
/// Response for PARSE_URDF
/// </summary>
public class ParseUrdfResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string? Error { get; set; }

    [JsonPropertyName("model")]
    public UrdfModelPayload? Model { get; set; }
}

/// <summary>
/// Request for GENERATE_ROBOT_YAML
/// </summary>
public class GenerateRobotYamlRequest
{
    [JsonPropertyName("urdf_content")]
    public string UrdfContent { get; set; } = "";

    [JsonPropertyName("is_file_path")]
    public bool IsFilePath { get; set; }

    [JsonPropertyName("robot_name")]
    public string RobotName { get; set; } = "";

    [JsonPropertyName("manufacturer")]
    public string Manufacturer { get; set; } = "Unknown";

    [JsonPropertyName("output_path")]
    public string OutputPath { get; set; } = "";
}

/// <summary>
/// Response for GENERATE_ROBOT_YAML
/// </summary>
public class GenerateRobotYamlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string? Error { get; set; }

    [JsonPropertyName("yaml_content")]
    public string YamlContent { get; set; } = "";

    [JsonPropertyName("saved_path")]
    public string? SavedPath { get; set; }
}
