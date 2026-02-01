namespace RobotController.Common.Services;

/// <summary>
/// Service for communicating with vision sensors
/// </summary>
public interface ISensorClientService
{
    // ========================================================================
    // Connection Events
    // ========================================================================

    event EventHandler<SensorConnectionEventArgs>? ConnectionChanged;
    event EventHandler<ProfileReceivedEventArgs>? ProfileReceived;
    event EventHandler<PointCloudReceivedEventArgs>? PointCloudReceived;
    event EventHandler<string>? ErrorOccurred;

    // ========================================================================
    // Laser Profiler
    // ========================================================================

    /// <summary>
    /// Enumerate available laser profilers
    /// </summary>
    Task<List<SensorInfo>> EnumerateLaserProfilersAsync();

    /// <summary>
    /// Connect to laser profiler
    /// </summary>
    Task<OperationResult> ConnectLaserAsync(string deviceId);

    /// <summary>
    /// Disconnect from laser profiler
    /// </summary>
    Task<OperationResult> DisconnectLaserAsync();

    /// <summary>
    /// Configure laser profiler
    /// </summary>
    Task<OperationResult> ConfigureLaserAsync(LaserProfilerConfigData config);

    /// <summary>
    /// Get laser profiler status
    /// </summary>
    Task<SensorInfoResponse> GetLaserStatusAsync();

    /// <summary>
    /// Start laser acquisition
    /// </summary>
    Task<OperationResult> StartLaserAcquisitionAsync();

    /// <summary>
    /// Stop laser acquisition
    /// </summary>
    Task<OperationResult> StopLaserAcquisitionAsync();

    /// <summary>
    /// Software trigger
    /// </summary>
    Task<OperationResult> SoftwareTriggerAsync();

    /// <summary>
    /// Get single profile
    /// </summary>
    Task<ProfileDataResponse> GetProfileAsync(uint timeoutMs = 1000);

    // ========================================================================
    // Scanning
    // ========================================================================

    /// <summary>
    /// Collect profiles for 3D scan
    /// </summary>
    Task<PointCloudResponse> CollectScanDataAsync(uint profileCount, uint timeoutMs = 30000);

    /// <summary>
    /// Downsample point cloud
    /// </summary>
    Task<PointCloudResponse> DownsamplePointCloudAsync(PointCloudData cloud, float voxelSize);

    // ========================================================================
    // Calibration
    // ========================================================================

    /// <summary>
    /// Perform hand-eye calibration
    /// </summary>
    Task<CalibrationResponse> PerformHandEyeCalibrationAsync(
        List<double[]> robotJoints,
        List<double[]> targetPoses);

    /// <summary>
    /// Save calibration to file
    /// </summary>
    Task<OperationResult> SaveCalibrationAsync(string filepath);

    /// <summary>
    /// Load calibration from file
    /// </summary>
    Task<OperationResult> LoadCalibrationAsync(string filepath);

    /// <summary>
    /// Get current calibration
    /// </summary>
    Task<CalibrationResponse> GetCalibrationAsync();
}

// ============================================================================
// Data Types
// ============================================================================

public record SensorInfo(
    string DeviceId,
    string SerialNumber,
    string ModelName,
    string Manufacturer,
    string Status);

public record SensorConnectionEventArgs(
    string DeviceId,
    bool Connected,
    string? Error = null);

public record ProfileReceivedEventArgs(
    float[] PointsX,
    float[] PointsZ,
    float[] Intensity,
    ulong Timestamp,
    ulong FrameId);

public record PointCloudReceivedEventArgs(
    float[] PointsX,
    float[] PointsY,
    float[] PointsZ,
    float[] Intensity,
    int PointCount);

public class LaserProfilerConfigData
{
    public uint ProfileWidth { get; set; } = 2048;
    public float ExposureTime { get; set; } = 1000.0f;
    public float Gain { get; set; } = 1.0f;
    public string TriggerMode { get; set; } = "freerun";
    public uint EncoderDivider { get; set; } = 1;
    public float IntensityLow { get; set; } = 10.0f;
    public float IntensityHigh { get; set; } = 250.0f;
}

public class PointCloudData
{
    public float[] PointsX { get; set; } = [];
    public float[] PointsY { get; set; } = [];
    public float[] PointsZ { get; set; } = [];
    public float[] Intensity { get; set; } = [];
    public int PointCount { get; set; }
}

public class OperationResult
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";

    public static OperationResult Ok(string message = "") => new() { Success = true, Message = message };
    public static OperationResult Fail(string message) => new() { Success = false, Message = message };
}

public class SensorInfoResponse
{
    public bool Success { get; set; }
    public string DeviceId { get; set; } = "";
    public string SerialNumber { get; set; } = "";
    public string ModelName { get; set; } = "";
    public string Status { get; set; } = "";
    public float FrameRate { get; set; }
    public ulong FramesReceived { get; set; }
    public ulong FramesDropped { get; set; }
    public string ErrorMessage { get; set; } = "";
}

public class ProfileDataResponse
{
    public bool Success { get; set; }
    public float[] PointsX { get; set; } = [];
    public float[] PointsZ { get; set; } = [];
    public float[] Intensity { get; set; } = [];
    public bool[] Valid { get; set; } = [];
    public ulong Timestamp { get; set; }
    public ulong FrameId { get; set; }
    public double[] RobotJoints { get; set; } = [];
    public string ErrorMessage { get; set; } = "";
}

public class PointCloudResponse
{
    public bool Success { get; set; }
    public float[] PointsX { get; set; } = [];
    public float[] PointsY { get; set; } = [];
    public float[] PointsZ { get; set; } = [];
    public float[] Intensity { get; set; } = [];
    public uint PointCount { get; set; }
    public float[] MinBound { get; set; } = [];
    public float[] MaxBound { get; set; } = [];
    public string FrameId { get; set; } = "";
    public string ErrorMessage { get; set; } = "";
}

public class CalibrationResponse
{
    public bool Success { get; set; }
    public double[] SensorToFlange { get; set; } = [];
    public double[] Translation { get; set; } = [];
    public double[] RotationQuat { get; set; } = [];
    public double ReprojectionError { get; set; }
    public string ErrorMessage { get; set; } = "";
}
