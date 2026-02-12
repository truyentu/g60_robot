using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

public class ComputeIKRequest
{
    [JsonPropertyName("target_pose")]
    public double[] TargetPose { get; set; } = new double[6]; // [x,y,z,rx,ry,rz] mm/deg

    [JsonPropertyName("current_joints")]
    public double[] CurrentJoints { get; set; } = new double[6]; // [j1..j6] degrees

    [JsonPropertyName("apply")]
    public bool Apply { get; set; } // If true, apply solved joints to firmware driver
}

public class ComputeIKResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("joints")]
    public double[] Joints { get; set; } = new double[6]; // [j1..j6] degrees

    [JsonPropertyName("residual_error")]
    public double ResidualError { get; set; }

    [JsonPropertyName("iterations")]
    public int Iterations { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}
