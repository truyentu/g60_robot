using System;
using System.Threading.Tasks;

namespace RobotController.Common.Services;

/// <summary>
/// Joint type enumeration matching C++ SeamTypes.hpp
/// </summary>
public enum JointType
{
    Unknown = 0,
    VGroove = 1,
    UGroove = 2,
    JGroove = 3,
    LapJoint = 4,
    FilletJoint = 5,
    ButtSquare = 6,
    EdgeJoint = 7,
    Custom = 8
}

/// <summary>
/// Seam feature data from detection
/// </summary>
public record SeamFeatureData
{
    public bool Success { get; init; }
    public JointType JointType { get; init; }
    public float RootX { get; init; }
    public float RootZ { get; init; }
    public float GapWidth { get; init; }
    public float LeftAngle { get; init; }
    public float RightAngle { get; init; }
    public float Depth { get; init; }
    public float Confidence { get; init; }
    public ulong Timestamp { get; init; }
    public string? ErrorMessage { get; init; }
}

/// <summary>
/// Seam point data in world coordinates
/// </summary>
public record SeamPointData
{
    public bool Success { get; init; }
    public float WorldX { get; init; }
    public float WorldY { get; init; }
    public float WorldZ { get; init; }
    public float[] Normal { get; init; } = new float[3];
    public float[] Tangent { get; init; } = new float[3];
    public float Deviation { get; init; }
    public float HeightOffset { get; init; }
    public ulong Timestamp { get; init; }
    public string? ErrorMessage { get; init; }
}

/// <summary>
/// Tracking state data
/// </summary>
public record TrackingStateData
{
    public bool Success { get; init; }
    public bool TrackingActive { get; init; }
    public bool TrackingLost { get; init; }
    public float PredictedX { get; init; }
    public float PredictedZ { get; init; }
    public float LateralCorrection { get; init; }
    public float HeightCorrection { get; init; }
    public float TrackingQuality { get; init; }
    public ulong FramesProcessed { get; init; }
    public ulong ValidDetections { get; init; }
    public float AvgLatencyMs { get; init; }
    public string? ErrorMessage { get; init; }
}

/// <summary>
/// Seam path data
/// </summary>
public record SeamPathData
{
    public bool Success { get; init; }
    public float[] PointsX { get; init; } = Array.Empty<float>();
    public float[] PointsY { get; init; } = Array.Empty<float>();
    public float[] PointsZ { get; init; } = Array.Empty<float>();
    public float[] Deviations { get; init; } = Array.Empty<float>();
    public float TotalLength { get; init; }
    public float AvgGapWidth { get; init; }
    public JointType JointType { get; init; }
    public string? ErrorMessage { get; init; }
}

/// <summary>
/// Tracking configuration
/// </summary>
public record TrackingConfig
{
    public bool EnableMedianFilter { get; init; } = true;
    public int MedianFilterSize { get; init; } = 5;
    public bool EnableOutlierRemoval { get; init; } = true;
    public float OutlierThreshold { get; init; } = 3.0f;
    public int RansacIterations { get; init; } = 100;
    public float RansacThreshold { get; init; } = 0.5f;
    public float SystemLatency { get; init; } = 0.030f;
    public float TravelSpeed { get; init; } = 10.0f;
}

/// <summary>
/// ROI configuration for seam tracking
/// </summary>
public record SeamRoiConfig
{
    public float XMin { get; init; } = -50.0f;
    public float XMax { get; init; } = 50.0f;
    public float ZMin { get; init; } = 0.0f;
    public float ZMax { get; init; } = 100.0f;
}

/// <summary>
/// Event args for seam feature updates
/// </summary>
public class SeamFeatureEventArgs : EventArgs
{
    public SeamFeatureData Feature { get; }

    public SeamFeatureEventArgs(SeamFeatureData feature)
    {
        Feature = feature;
    }
}

/// <summary>
/// Event args for tracking state updates
/// </summary>
public class TrackingStateEventArgs : EventArgs
{
    public TrackingStateData State { get; }

    public TrackingStateEventArgs(TrackingStateData state)
    {
        State = state;
    }
}

/// <summary>
/// Interface for seam detection service
/// </summary>
public interface ISeamDetectionService
{
    /// <summary>
    /// Event raised when a new seam feature is detected
    /// </summary>
    event EventHandler<SeamFeatureEventArgs>? FeatureDetected;

    /// <summary>
    /// Event raised when tracking state changes
    /// </summary>
    event EventHandler<TrackingStateEventArgs>? TrackingStateChanged;

    /// <summary>
    /// Start seam tracking with specified joint type
    /// </summary>
    Task<bool> StartTrackingAsync(JointType jointType, SeamRoiConfig? roi = null);

    /// <summary>
    /// Stop seam tracking
    /// </summary>
    Task<bool> StopTrackingAsync();

    /// <summary>
    /// Set tracking configuration
    /// </summary>
    Task<bool> SetTrackingConfigAsync(TrackingConfig config);

    /// <summary>
    /// Get current tracking state
    /// </summary>
    Task<TrackingStateData> GetTrackingStateAsync();

    /// <summary>
    /// Get latest seam feature
    /// </summary>
    Task<SeamFeatureData> GetLatestFeatureAsync();

    /// <summary>
    /// Get current seam point in world coordinates
    /// </summary>
    Task<SeamPointData> GetCurrentSeamPointAsync();

    /// <summary>
    /// Set nominal path for deviation calculation
    /// </summary>
    Task<bool> SetNominalPathAsync(float[] pointsX, float[] pointsY, float[] pointsZ);

    /// <summary>
    /// Get detected seam path
    /// </summary>
    Task<SeamPathData> GetSeamPathAsync();

    /// <summary>
    /// Reset tracker state
    /// </summary>
    Task<bool> ResetTrackerAsync();

    /// <summary>
    /// Check if tracking is currently active
    /// </summary>
    bool IsTrackingActive { get; }

    /// <summary>
    /// Current tracking quality (0-1)
    /// </summary>
    float TrackingQuality { get; }
}
