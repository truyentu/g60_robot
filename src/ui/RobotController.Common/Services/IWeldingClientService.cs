using RobotController.Common.Messages;

namespace RobotController.Common.Services;

/// <summary>
/// Client service for welding control
/// </summary>
public interface IWeldingClientService
{
    // ========================================================================
    // Control
    // ========================================================================

    /// <summary>
    /// Start welding with specified job parameters
    /// </summary>
    Task<WeldingControlResponse> StartWeldAsync(WeldingJobData job, CancellationToken ct = default);

    /// <summary>
    /// Stop welding (normal end with crater fill)
    /// </summary>
    Task<WeldingControlResponse> StopWeldAsync(CancellationToken ct = default);

    /// <summary>
    /// Abort welding immediately
    /// </summary>
    Task<WeldingControlResponse> AbortWeldAsync(CancellationToken ct = default);

    /// <summary>
    /// Reset from fault state
    /// </summary>
    Task<WeldingControlResponse> ResetAsync(CancellationToken ct = default);

    // ========================================================================
    // Status
    // ========================================================================

    /// <summary>
    /// Get current welding status
    /// </summary>
    Task<WeldingStatusResponse> GetStatusAsync(CancellationToken ct = default);

    /// <summary>
    /// Last known welding status
    /// </summary>
    WeldingStatusResponse CachedStatus { get; }

    // ========================================================================
    // Parameters
    // ========================================================================

    /// <summary>
    /// Adjust welding current during welding
    /// </summary>
    Task<WeldingAdjustResponse> AdjustCurrentAsync(float current, CancellationToken ct = default);

    /// <summary>
    /// Adjust arc voltage during welding
    /// </summary>
    Task<WeldingAdjustResponse> AdjustVoltageAsync(float voltage, CancellationToken ct = default);

    /// <summary>
    /// Adjust wire feed speed during welding
    /// </summary>
    Task<WeldingAdjustResponse> AdjustWireSpeedAsync(float speed, CancellationToken ct = default);

    /// <summary>
    /// Adjust travel speed during welding
    /// </summary>
    Task<WeldingAdjustResponse> AdjustTravelSpeedAsync(float speed, CancellationToken ct = default);

    // ========================================================================
    // Events
    // ========================================================================

    /// <summary>
    /// Fired when welding status is updated
    /// </summary>
    event EventHandler<WeldingStatusResponse>? StatusUpdated;

    /// <summary>
    /// Fired when a welding fault occurs
    /// </summary>
    event EventHandler<WeldingFault>? FaultOccurred;

    /// <summary>
    /// Fired when welding state changes
    /// </summary>
    event EventHandler<WeldingState>? StateChanged;
}
